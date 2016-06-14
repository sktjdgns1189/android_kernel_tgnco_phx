ccflags-y += -Idrivers/media/platform/msm/camera_v2/
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ifndef CONFIG_FIH_ISP_MAIN_CAM
obj-$(CONFIG_MSM_CCI) += msm_cci.o
endif
obj-$(CONFIG_FIH_ISP_MAIN_CAM) += msm_cci_external_isp.o
