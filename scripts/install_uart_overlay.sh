#!/bin/bash
set -e

echo "=== 1. Biên dịch Dual UART Device Tree Overlay ==="
cat << 'DT_EOF' > /tmp/meson-all-uarts.dts
/dts-v1/;
/plugin/;

/ {
	fragment@0 {
		target-path = "/soc/bus@ffd00000/serial@23000";
		__overlay__ {
			status = "okay";
			pinctrl-0 = <&uart_b_pins>;
			pinctrl-names = "default";
		};
	};

	fragment@1 {
		target-path = "/soc/bus@ff800000/serial@4000";
		__overlay__ {
			status = "okay";
			pinctrl-0 = <&uart_ao_b_8_9_pins>;
			pinctrl-names = "default";
		};
	};
};
DT_EOF

dtc -@ -I dts -O dtb /tmp/meson-all-uarts.dts -o /boot/dtb/amlogic/overlay/meson-uartB.dtbo
chmod 644 /boot/dtb/amlogic/overlay/meson-uartB.dtbo
echo "[OK] Đã cài đặt /boot/dtb/amlogic/overlay/meson-uartB.dtbo"

echo "=== 2. Kiểm tra /boot/armbianEnv.txt ==="
if ! grep -q "^overlays=" /boot/armbianEnv.txt; then
    echo "overlays=uartB" >> /boot/armbianEnv.txt
else
    if ! grep -q "uartB" /boot/armbianEnv.txt; then
        sed -i 's/^overlays=.*/& uartB/' /boot/armbianEnv.txt
    fi
fi
cat /boot/armbianEnv.txt

echo "=========================================================="
echo "[THÀNH CÔNG] Hãy chạy lệnh: sudo reboot"
echo "=========================================================="
