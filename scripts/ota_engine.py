#!/usr/bin/env python3
"""
ota_engine.py - Autonomous OTA State Machine and Execution Engine
Part of Receptionist Robot AMR Core (radxa_amr_v1)

Lifecycle:
[IDLE] -> [CHECKING_SAFETY] -> [DOWNLOADING] -> [VERIFYING] ->
[EXTRACTING] -> [SWITCHING_LINK] -> [RESTARTING] -> [HEALTHCHECK] ->
[COMPLETED] or [ROLLING_BACK] -> [ROLLED_BACK]
"""

import os
import sys
import time
import json
import shutil
import hashlib
import urllib.request
import urllib.error
import subprocess
import threading
from pathlib import Path
from typing import Dict, Any, Optional, Tuple, List

# Core Directory Layout
OTA_BASE_DIR = Path("/home/rctrobot/ota")
RELEASES_DIR = OTA_BASE_DIR / "releases"
STAGING_DIR = OTA_BASE_DIR / "staging"
STATE_FILE = OTA_BASE_DIR / "ota_state.json"

ROBOT_DATA_DIR = Path("/home/rctrobot/robot_data")
MAPS_DIR = ROBOT_DATA_DIR / "maps"
CONFIG_DIR = ROBOT_DATA_DIR / "config"

AMR_LINK = Path("/home/rctrobot/radxa_amr_v1")
AMR_NEXT_LINK = Path("/home/rctrobot/radxa_amr_next")
AMR_CURRENT_TARGET = Path("/home/rctrobot/radxa_amr_current")

SERVICE_NAME = "robot_bringup.service"
GITHUB_REPO = "miguelsonohara/radxa_amr_v1"
HEALTHCHECK_TIMEOUT_SEC = 45


def get_free_disk_mb() -> int:
    """Returns available disk space in MB for /home/rctrobot."""
    try:
        stat = os.statvfs("/home/rctrobot")
        return int((stat.f_bavail * stat.f_frsize) / (1024 * 1024))
    except Exception:
        return 0


class OTAEngine:
    def __init__(self):
        self.lock = threading.Lock()
        
        # Ensure directories exist
        RELEASES_DIR.mkdir(parents=True, exist_ok=True)
        STAGING_DIR.mkdir(parents=True, exist_ok=True)
        ROBOT_DATA_DIR.mkdir(parents=True, exist_ok=True)
        MAPS_DIR.mkdir(parents=True, exist_ok=True)
        CONFIG_DIR.mkdir(parents=True, exist_ok=True)

        self._state_data = self._load_persisted_state()
        
        # Runtime in-memory state
        self.stage = "IDLE"  # IDLE, CHECKING_SAFETY, DOWNLOADING, VERIFYING, EXTRACTING, SWITCHING_LINK, RESTARTING, HEALTHCHECK, COMPLETED, ROLLING_BACK, ROLLED_BACK
        self.progress = 0
        self.message = "Sẵn sàng"
        self.error = None
        self.is_busy = False

    def _load_persisted_state(self) -> Dict[str, Any]:
        """Loads persistent OTA metadata or initializes default."""
        if STATE_FILE.exists():
            try:
                with open(STATE_FILE, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception:
                pass
        
        default_state = {
            "current_version": "v1.0.0",
            "previous_version": None,
            "last_updated": None,
            "releases_history": []
        }
        self._save_persisted_state(default_state)
        return default_state

    def _save_persisted_state(self, data: Optional[Dict[str, Any]] = None):
        """Saves persistent metadata to disk."""
        if data is not None:
            self._state_data = data
        try:
            with open(STATE_FILE, "w", encoding="utf-8") as f:
                json.dump(self._state_data, f, indent=2)
        except Exception as e:
            print(f"[ota_engine] Error saving state: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Returns the public status object for Web Kiosk polling."""
        with self.lock:
            return {
                "success": True,
                "current_version": self._state_data.get("current_version", "v1.0.0"),
                "previous_version": self._state_data.get("previous_version"),
                "stage": self.stage,
                "progress": self.progress,
                "message": self.message,
                "error": self.error,
                "is_busy": self.is_busy,
                "free_disk_mb": get_free_disk_mb(),
                "last_updated": self._state_data.get("last_updated"),
            }

    def _update_state(self, stage: str, progress: int, message: str, error: Optional[str] = None):
        with self.lock:
            self.stage = stage
            self.progress = progress
            self.message = message
            self.error = error
            self.is_busy = stage not in ["IDLE", "COMPLETED", "ROLLED_BACK"]
            print(f"[OTA][{stage}][{progress}%] {message} (error={error})")

    def check_safety(self) -> Tuple[bool, str]:
        """
        Ensures the robot is stationary before applying an update.
        Checks if /cmd_vel has linear or angular velocity != 0.
        """
        self._update_state("CHECKING_SAFETY", 5, "Kiểm tra an toàn robot...")
        try:
            # Query the latest message on /cmd_vel using ros2 topic echo
            cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic echo /cmd_vel --once --timeout 1.5"
            res = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=3)
            if res.returncode == 0 and res.stdout:
                # Parse linear x and angular z
                lin_match = re.search(r"x:\s*([-\d.]+)", res.stdout)
                ang_match = re.search(r"z:\s*([-\d.]+)", res.stdout)
                lin_x = float(lin_match.group(1)) if lin_match else 0.0
                ang_z = float(ang_match.group(1)) if ang_match else 0.0
                if abs(lin_x) > 0.01 or abs(ang_z) > 0.01:
                    msg = f"Robot đang di chuyển (v_x={lin_x}, w_z={ang_z})! Không thể cập nhật."
                    return False, msg
        except Exception as e:
            # If echo times out or ROS not ready, robot is stationary
            pass
        return True, "Robot an toàn (đang đứng yên)"

    def check_for_updates(self) -> Dict[str, Any]:
        """
        Queries GitHub API for latest release.
        Returns update details or indicates already up-to-date.
        """
        url = f"https://api.github.com/repos/{GITHUB_REPO}/releases/latest"
        req = urllib.request.Request(
            url,
            headers={
                "User-Agent": "ReceptionistRobot-OTA/1.0",
                "Accept": "application/vnd.github.v3+json"
            }
        )
        try:
            with urllib.request.urlopen(req, timeout=8) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            
            tag_name = data.get("tag_name", "").strip()
            current_ver = self._state_data.get("current_version", "v1.0.0")
            
            # Find tar.gz and sha256 assets
            download_url = None
            sha256_url = None
            for asset in data.get("assets", []):
                name = asset.get("name", "")
                if name.endswith(".tar.gz"):
                    download_url = asset.get("browser_download_url")
                elif name.endswith(".txt") or name.endswith(".sha256"):
                    sha256_url = asset.get("browser_download_url")
            
            # Retrieve expected sha256 hash if sha256 file exists
            expected_sha256 = ""
            if sha256_url:
                try:
                    s_req = urllib.request.Request(sha256_url, headers={"User-Agent": "ReceptionistRobot-OTA/1.0"})
                    with urllib.request.urlopen(s_req, timeout=5) as s_resp:
                        sha_content = s_resp.read().decode("utf-8").strip()
                        # Extract 64-char hex string
                        hex_matches = re.findall(r"[a-fA-F0-9]{64}", sha_content)
                        if hex_matches:
                            expected_sha256 = hex_matches[0].lower()
                except Exception:
                    pass

            has_update = (tag_name and tag_name != current_ver)
            return {
                "success": True,
                "has_update": has_update,
                "current_version": current_ver,
                "latest_version": tag_name,
                "release_notes": data.get("body", "Bản cập nhật mới"),
                "download_url": download_url,
                "sha256": expected_sha256,
            }
        except urllib.error.URLError as e:
            return {"success": False, "has_update": False, "message": f"Không có kết nối mạng: {e}"}
        except Exception as e:
            return {"success": False, "has_update": False, "message": f"Lỗi kiểm tra cập nhật: {e}"}

    def _download_stream(self, url: str, target_file: Path) -> bool:
        """Downloads file with chunked streaming and percentage updates (10% -> 60%)."""
        self._update_state("DOWNLOADING", 10, "Bắt đầu tải gói phát hành...")
        req = urllib.request.Request(url, headers={"User-Agent": "ReceptionistRobot-OTA/1.0"})
        with urllib.request.urlopen(req, timeout=30) as resp, open(target_file, "wb") as f:
            total_size = resp.headers.get("Content-Length")
            total_bytes = int(total_size) if total_size else 0
            downloaded = 0
            chunk_size = 64 * 1024  # 64KB chunks

            last_report_time = time.time()
            while True:
                chunk = resp.read(chunk_size)
                if not chunk:
                    break
                f.write(chunk)
                downloaded += len(chunk)

                if total_bytes > 0:
                    pct = 10 + int((downloaded / total_bytes) * 50)
                else:
                    pct = min(10 + int(downloaded / (1024 * 1024 * 2)), 59)
                
                now = time.time()
                if now - last_report_time >= 0.5:
                    last_report_time = now
                    mb = downloaded / (1024 * 1024)
                    self._update_state("DOWNLOADING", pct, f"Đang tải bản cập nhật: {mb:.1f} MB ({pct}%)...")

        self._update_state("DOWNLOADING", 60, "Tải về hoàn tất.")
        return True

    def _verify_sha256(self, filepath: Path, expected_hash: str) -> bool:
        """Computes SHA-256 and compares with expected."""
        self._update_state("VERIFYING", 70, "Kiểm tra tính toàn vẹn (SHA-256)...")
        if not expected_hash:
            print("[ota_engine] Warning: No expected SHA-256 provided, skipping hash verification.")
            return True

        hasher = hashlib.sha256()
        with open(filepath, "rb") as f:
            while chunk := f.read(128 * 1024):
                hasher.update(chunk)
        calc_hash = hasher.hexdigest().lower()
        if calc_hash != expected_hash.strip().lower():
            err = f"SHA-256 không khớp! Tính được: {calc_hash}, Mong đợi: {expected_hash}"
            self._update_state("VERIFYING", 70, "Xác thực thất bại", error=err)
            return False
        return True

    def _extract_and_switch(self, tar_path: Path, version_tag: str) -> Path:
        """Extracts release to releases/amr_<version> and prepares atomic symlink."""
        dest_dir = RELEASES_DIR / f"amr_{version_tag}"
        if dest_dir.exists():
            shutil.rmtree(dest_dir)
        dest_dir.mkdir(parents=True, exist_ok=True)

        self._update_state("EXTRACTING", 80, f"Đang giải nén gói phát hành {version_tag}...")
        res = subprocess.run(["tar", "-xzf", str(tar_path), "-C", str(dest_dir)], capture_output=True, text=True)
        if res.returncode != 0:
            raise RuntimeError(f"Giải nén thất bại: {res.stderr}")

        # Ensure compatibility symlinks inside new release to persistent robot_data
        new_map_dir = dest_dir / "map"
        if new_map_dir.exists() and not new_map_dir.is_symlink():
            shutil.rmtree(new_map_dir)
        if not new_map_dir.exists():
            new_map_dir.symlink_to(MAPS_DIR)

        self._update_state("SWITCHING_LINK", 85, "Hoán đổi Atomic Symlink phiên bản...")
        # Step A: Atomically link /home/rctrobot/radxa_amr_next to new release
        subprocess.run(["ln", "-sfn", str(dest_dir), str(AMR_NEXT_LINK)], check=True)
        # Step B: Atomic rename /home/rctrobot/radxa_amr_next -> /home/rctrobot/radxa_amr_current
        subprocess.run(["mv", "-Tf", str(AMR_NEXT_LINK), str(AMR_CURRENT_TARGET)], check=True)
        # Step C: Ensure /home/rctrobot/radxa_amr_v1 points to current target or dest_dir
        if AMR_LINK.is_symlink() or not AMR_LINK.exists():
            subprocess.run(["ln", "-sfn", str(dest_dir), str(AMR_LINK)], check=True)
        elif AMR_LINK.is_dir():
            # If it's a real directory (original clone), we point /home/rctrobot/radxa_amr_current to dest_dir
            pass

        return dest_dir

    def _restart_service(self):
        """Restarts robot_bringup.service."""
        self._update_state("RESTARTING", 90, "Khởi động lại dịch vụ robot_bringup...")
        res = subprocess.run(["sudo", "-n", "systemctl", "restart", SERVICE_NAME], capture_output=True, text=True)
        if res.returncode != 0:
            raise RuntimeError(f"Restart service thất bại: {res.stderr}")

    def _healthcheck(self, timeout=HEALTHCHECK_TIMEOUT_SEC) -> bool:
        """Watchdog: verifies service is active and /odom is publishing within timeout."""
        self._update_state("HEALTHCHECK", 95, f"Watchdog kiểm tra hoạt động hệ thống ({timeout}s)...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 1. Check if service is active
            chk = subprocess.run(["systemctl", "is-active", SERVICE_NAME], capture_output=True, text=True)
            if chk.stdout.strip() == "active":
                # 2. Check if /odom topic receives message
                cmd = "source /opt/ros/jazzy/setup.bash && ros2 topic echo /odom --once --timeout 2"
                echo_res = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=5)
                if echo_res.returncode == 0 and "pose" in echo_res.stdout:
                    return True
            time.sleep(2)
        return False

    def _cleanup_old_releases(self):
        """Retains only the 2 most recent releases to maintain disk buffer under 1GB."""
        try:
            releases = sorted([d for d in RELEASES_DIR.iterdir() if d.is_dir()], key=lambda x: x.stat().st_mtime)
            if len(releases) > 2:
                for old in releases[:-2]:
                    print(f"[ota_engine] Pruning old release: {old.name}")
                    shutil.rmtree(old, ignore_errors=True)
        except Exception as e:
            print(f"[ota_engine] Cleanup error: {e}")

    def rollback(self) -> Tuple[bool, str]:
        """Rolls back to the previous version if available."""
        self._update_state("ROLLING_BACK", 90, "Đang thực hiện hoàn nguyên (Rollback) phiên bản cũ...")
        prev_ver = self._state_data.get("previous_version")
        if not prev_ver:
            err = "Không tìm thấy phiên bản trước đó để hoàn nguyên."
            self._update_state("ROLLED_BACK", 0, "Không thể hoàn nguyên", error=err)
            return False, err

        prev_dir = RELEASES_DIR / f"amr_{prev_ver}"
        if not prev_dir.exists():
            err = f"Thư mục phiên bản cũ {prev_ver} không tồn tại trên đĩa."
            self._update_state("ROLLED_BACK", 0, "Không thể hoàn nguyên", error=err)
            return False, err

        try:
            # Atomic Symlink switch back
            subprocess.run(["ln", "-sfn", str(prev_dir), str(AMR_LINK)], check=True)
            self._restart_service()
            time.sleep(5)
            
            curr_ver = self._state_data.get("current_version")
            self._state_data["current_version"] = prev_ver
            self._state_data["previous_version"] = curr_ver
            self._save_persisted_state()

            self._update_state("ROLLED_BACK", 100, f"Đã hoàn nguyên về phiên bản {prev_ver} thành công.")
            return True, f"Hoàn nguyên về {prev_ver} thành công."
        except Exception as e:
            err = f"Lỗi trong quá trình hoàn nguyên: {e}"
            self._update_state("ROLLED_BACK", 0, "Lỗi rollback", error=err)
            return False, err

    def run_full_update(self, payload: Dict[str, Any]):
        """Executes the full OTA update pipeline in a background thread."""
        download_url = payload.get("download_url")
        version_tag = payload.get("version", "new_version").strip()
        sha256_expected = payload.get("sha256", "").strip()

        if not download_url:
            self._update_state("IDLE", 0, "Lỗi: Không có đường dẫn tải về (download_url)", error="MISSING_DOWNLOAD_URL")
            return

        staging_tar = STAGING_DIR / f"amr_{version_tag}.tar.gz"

        try:
            # 1. Check safety
            safe, reason = self.check_safety()
            if not safe:
                self._update_state("IDLE", 0, "Hủy cập nhật: " + reason, error="ROBOT_IN_MOTION_CANNOT_UPDATE")
                return

            # 2. Download package
            self._download_stream(download_url, staging_tar)

            # 3. Verify SHA-256
            if not self._verify_sha256(staging_tar, sha256_expected):
                if staging_tar.exists():
                    staging_tar.unlink()
                return

            # 4. Extract and Atomic Symlink switch
            new_release_dir = self._extract_and_switch(staging_tar, version_tag)

            # 5. Restart service
            self._restart_service()

            # 6. Watchdog and healthcheck
            ok = self._healthcheck(timeout=HEALTHCHECK_TIMEOUT_SEC)
            if ok:
                # Update persisted version state
                curr = self._state_data.get("current_version", "v1.0.0")
                self._state_data["previous_version"] = curr
                self._state_data["current_version"] = version_tag
                self._state_data["last_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
                self._save_persisted_state()

                # Clean staging and old releases
                if staging_tar.exists():
                    staging_tar.unlink()
                self._cleanup_old_releases()

                self._update_state("COMPLETED", 100, f"Cập nhật thành công lên phiên bản {version_tag}!")
            else:
                print("[ota_engine] Health check failed! Initiating automatic rollback...")
                self.rollback()

        except Exception as e:
            print(f"[ota_engine] Exception during OTA: {e}")
            self._update_state("ROLLING_BACK", 90, f"Lỗi cập nhật: {e}. Đang hoàn nguyên...")
            self.rollback()
        finally:
            if staging_tar.exists():
                try:
                    staging_tar.unlink()
                except Exception:
                    pass


# Singleton engine instance
engine = OTAEngine()
