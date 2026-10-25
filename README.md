# b43

Standalone Linux kernel module for Broadcom 43xx wireless devices.
Requires matching kernel headers with `CONFIG_B43=m`, kernel build tools,
and pahole for BTF-enabled kernels. Features follow the target kernel's configuration.

```sh
make -j4                  # builds b43.ko for the running kernel
sudo make modules_install
sudo depmod -a
make clean
```

For another kernel, pass `KDIR=/path/to/kernel/build` to make.
Firmware is installed separately using b43-fwcutter. GPL-2.0; see `COPYING`.
