#
# Copyright (C) 2008-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

DEVICE_VARS += IMAGE_SIZE 

KERNEL_LOADADDR = 0x80000000
KERNEL_ENTRY = 0x80000400

#KERNEL_DTB = kernel-bin | append-dtb | gzip
define Device/Default
  PROFILES = Default
  KERNEL := kernel-bin | append-dtb | gzip | uImage gzip
  KERNEL_INITRAMFS := kernel-bin | append-dtb | gzip | uImage gzip
#  KERNEL := kernel-bin | gzip | uImage gzip
  DEVICE_DTS_DIR := ../dts
  DEVICE_DTS = $(1)
  IMAGES := sysupgrade.bin
  SUPPORTED_DEVICES := $(subst _,$(comma),$(1))
  sysupgrade_bin := append-kernel | append-rootfs | pad-rootfs
  IMAGE/sysupgrade.bin := append-kernel | append-rootfs | pad-rootfs | append-metadata | check-size $$$$(IMAGE_SIZE)
endef


define Build/elecom-header
        cp $@ $(KDIR)/v_0.0.0.bin
        ( \
                mkhash md5 $(KDIR)/v_0.0.0.bin && \
                echo 458 \
        ) | mkhash md5 > $(KDIR)/v_0.0.0.md5
        $(STAGING_DIR_HOST)/bin/tar -c \
                $(if $(SOURCE_DATE_EPOCH),--mtime=@$(SOURCE_DATE_EPOCH)) \
                --owner=0 --group=0 -f $@ -C $(KDIR) v_0.0.0.bin v_0.0.0.md5
endef


define Device/zyxel_gs1900-10hp
  SOC := rtl838x
  IMAGE_SIZE := 7936k
  DEVICE_VENDOR := Zyxel
  DEVICE_MODEL := GS1900-10HP
  DEVICE_PACKAGES:= kmod-usb2 kmod-usb-ohci
  SUPPORTED_DEVICES += gs1900-10hp
endef
TARGET_DEVICES += zyxel_gs1900-10hp

define Device/d-link_dgs-1210-16
  SOC := rtl838x
  IMAGE_SIZE := 7936k
  DEVICE_VENDOR := D-Link
  DEVICE_MODEL := DGS-1210-16
  DEVICE_PACKAGES:= kmod-usb2 kmod-usb-ohci
  SUPPORTED_DEVICES += dgs-1210-16
endef
TARGET_DEVICES += d-link_dgs-1210-16

define Device/tp-link_t2500g-10ts
  SOC := rtl838x
  IMAGE_SIZE := 7936k
  DEVICE_VENDOR := TP-Link
  DEVICE_MODEL := T2500G-10TS
  DEVICE_PACKAGES:= kmod-usb2 kmod-usb-ohci
  SUPPORTED_DEVICES += t2500g-10ts
endef
TARGET_DEVICES += tp-link_t2500g-10ts



$(eval $(call BuildImage))
