################################################################################
#
# closed-media - Closed source binary blocs to enable DSP on Neuros OSD
#
################################################################################

CLOSED_MEDIA_VERSION:=1852
CLOSED_MEDIA_SOURCE:=closed-media-$(CLOSED_MEDIA_VERSION).tar.gz
CLOSED_MEDIA_SITE:=http://trac.neurostechnology.com/neuros-bsp/export/$(CLOSED_MEDIA_VERSION)/trunk
CLOSED_MEDIA_MODULE_SITE:=$(CLOSED_MEDIA_SITE)/kernels/kos_64M
CLOSED_MEDIA_MODULE_FILES:=pcm.ko mpeg2_pcm_dec.ko mpeg4_pcm_dec.ko mpeg4sp_g7xx_dec.ko \
divx311_pcm_dec.ko wmv_wma_dec.ko divx311.ko mpeg2.ko wmv.ko mpeg4_pcm_enc.ko \
mpeg4sp_g7xx_enc.ko mpeg4sp_g7xx_codec.ko image_dec.ko image_enc.ko jpeg_codec.ko \
jpeg_pcm_dec.ko image.ko imanage.ko ividio.ko iaudio.ko idecode.ko iencode.ko
CLOSED_MEDIA_HEADER_SITE:=$(CLOSED_MEDIA_SITE)/toolchain/arm-linux/include/linux
CLOSED_MEDIA_HEADER_FILES:=imanage.h imanage_ioctl.h icapture.h ividio.h idecode.h \
idecode_ioctl.h iencode.h iencode_ioctl.h ierror.h iqueue.h codecmap.h mcbsp.h
CLOSED_MEDIA_DIR:=$(BUILD_DIR)/closed-media
CLOSED_MEDIA_TARGET_MODULE_PATH:=lib/modules/$(LINUX26_VERSION_PROBED)/extra


# Stick header files & modules all into a tarball
$(DL_DIR)/$(CLOSED_MEDIA_SOURCE): 
	@$(call MESSAGE,"closed-media - Downloading files")
	# Get header files individually via http
	cd $(DL_DIR) && \
	for i in $(CLOSED_MEDIA_MODULE_FILES); \
	do \
		$(WGET) -P $(DL_DIR) $(CLOSED_MEDIA_MODULE_SITE)/$$i; \
	done

	# Get modules individually via http
	cd $(DL_DIR) && \
	for i in $(CLOSED_MEDIA_HEADER_FILES); \
	do \
		$(WGET) -P $(DL_DIR) $(CLOSED_MEDIA_HEADER_SITE)/$$i; \
	done

	#combine the module checkout with header files already obtained
	cd $(DL_DIR) && \
	$(TAR) -cf $(basename $(CLOSED_MEDIA_SOURCE)) $(CLOSED_MEDIA_HEADER_FILES) \
		$(CLOSED_MEDIA_MODULE_FILES) && \
	gzip -f $(basename $(CLOSED_MEDIA_SOURCE))
	#Clean up
	cd $(DL_DIR) && rm $(CLOSED_MEDIA_HEADER_FILES) $(CLOSED_MEDIA_MODULE_FILES)

# Extract tarball containing modules and header files
$(CLOSED_MEDIA_DIR)/.source: $(DL_DIR)/$(CLOSED_MEDIA_SOURCE)
	echo $(DL_DIR)/$(CLOSED_MEDIA_SOURCE)
	@$(call MESSAGE,"closed-media - Extracting archive")
	mkdir -p $(CLOSED_MEDIA_DIR)
	$(ZCAT) $(DL_DIR)/$(CLOSED_MEDIA_SOURCE) | tar -C $(CLOSED_MEDIA_DIR) $(TAR_OPTIONS) -
	touch $@

# Correct binary modules to accept gcc version kernel is built with
$(CLOSED_MEDIA_DIR)/.configured: $(CLOSED_MEDIA_DIR)/.source
	@$(call MESSAGE,"closed-media - Patching")

	#Perform correction to all modules
	cd $(CLOSED_MEDIA_DIR) && \
	for i in *.ko; \
	do \
	if test -f "$$i"; \
		then \
		sed s/gcc-3.4/gcc-4.4/ "$$i" > "$$i.new"; \
		rm "$$i"; \
		mv "$$i.new" "$$i"; \
	fi; \
	done
	touch $@

$(CLOSED_MEDIA_DIR)/.installed: $(CLOSED_MEDIA_DIR)/.configured
	touch $@

closed-media: $(CLOSED_MEDIA_DIR)/.installed
# Install modules to target
	@$(call MESSAGE,"closed-media - Installing modules to target")
	mkdir -p $(TARGET_DIR)/$(CLOSED_MEDIA_TARGET_MODULE_PATH)
	cd $(CLOSED_MEDIA_DIR) && \
	for i in $(PACKAGE_OSDRE_MEDIA-y); \
	do \
		cp "$$i" $(TARGET_DIR)/$(CLOSED_MEDIA_TARGET_MODULE_PATH)/; \
	done

# Install header files
	@$(call MESSAGE,"closed-media - Installing headers to staging")
	cd $(CLOSED_MEDIA_DIR) && \
	for i in $(PACKAGE_OSDRE_MEDIA_H-y); \
	do \
		cp "$$i" $(STAGING_DIR)/usr/include/linux/; \
	done

	@$(call MESSAGE,"closed-media - Installation complete")

closed-media-source: $(DL_DIR)/$(CLOSED_MEDIA_SOURCE)

closed-media-clean: closed-media-dirclean closed-media-uninstall

closed-media-uninstall:
	rm -rf $(TARGET_DIR)/$(CLOSED_MEDIA_TARGET_MODULE_PATH)
	rm -rf $(addprefix $(STAGING_DIR)/usr/include/linux/, $(CLOSED_MEDIA_HEADER_FILES))

closed-media-dirclean:
	rm -rf $(CLOSED_MEDIA_DIR)


################################################################################
#
# Toplevel Makefile options
#
################################################################################

# What modules to include?
PACKAGE_OSDRE_MEDIA-y:=
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_PCM) += pcm.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_MPEG2_PCM) += mpeg2_pcm_dec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_MPEG4_PCM) += mpeg4_pcm_dec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_MPEG4_G711) += mpeg4sp_g7xx_dec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_DIVX_PCM) += divx311_pcm_dec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_WMV_WMA_PCM) += wmv_wma_dec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_DIVX) += divx311.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_MPEG2) += mpeg2.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_WMV) += wmv.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_ENC_MPEG4_PCM) += mpeg4_pcm_enc.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_ENC_MPEG4_G711) += mpeg4sp_g7xx_enc.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_ENCDEC_MPEG4_G711) += mpeg4sp_g7xx_codec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_JPEG) += image_dec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_ENC_JPEG) += image_enc.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_ENCDEC_JPEG) += jpeg_codec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_JPEG_PCM) += jpeg_pcm_dec.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_DEC_IMAGE) += image.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IMANAGE) += imanage.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IVIDIO) += ividio.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IAUDIO) += iaudio.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IDECODE) += idecode.ko
PACKAGE_OSDRE_MEDIA-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IENCODE) += iencode.ko

# What header files to include?
PACKAGE_OSDRE_MEDIA_H-y:=
PACKAGE_OSDRE_MEDIA_H-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IMANAGE) += imanage.h imanage_ioctl.h
PACKAGE_OSDRE_MEDIA_H-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IVIDIO) += icapture.h ividio.h
PACKAGE_OSDRE_MEDIA_H-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IDECODE) += idecode.h idecode_ioctl.h
PACKAGE_OSDRE_MEDIA_H-$(BR2_PACKAGE_OSDRE_CLOSED_MEDIA_IENCODE) += iencode.h iencode_ioctl.h


# If anything above required, add this makefile to the build process
ifneq ($(PACKAGE_OSDRE_CLOSED_MEDIA-y),"")
TARGETS+=closed-media

#Add other required header files
PACKAGE_OSDRE_MEDIA_H-y += ierror.h iqueue.h codecmap.h mcbsp.h
endif

