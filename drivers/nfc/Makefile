#
# Makefile for nfc devices
#

obj-$(CONFIG_PN544_NFC)		+= pn544.o
obj-$(CONFIG_NXP_PN544_NFC)	+= nxp_pn544.o
obj-$(CONFIG_NXP_PN547_NFC)	+= nxp_pn547.o
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
