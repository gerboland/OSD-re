################################################################################
#
# closed-hwdrivers - Closed source binary blocs to enable hardware on Neuros OSD
#
################################################################################

CLOSED_HWDRIVERS_VERSION:=1852
CLOSED_HWDRIVERS_SOURCE:=closed-hwdrivers-$(CLOSED_HWDRIVERS_VERSION).tar.gz
CLOSED_HWDRIVERS_SITE:=http://trac.neurostechnology.com/neuros-bsp/export/$(CLOSED_HWDRIVERS_VERSION)/trunk
CLOSED_HWDRIVERS_MODULE_SITE:=$(CLOSED_HWDRIVERS_SITE)/kernels/kos_64M
CLOSED_HWDRIVERS_MODULE_FILES:=neuros_rtc.ko tvp5150.ko aic23.ko itfb.ko
CLOSED_HWDRIVERS_HEADER_SITE:=$(CLOSED_HWDRIVERS_SITE)/toolchain/arm-linux/include/linux
CLOSED_HWDRIVERS_HEADER_FILES:=itfb.h neuros_ir_blaster.h neuros_ir.h aic23.h neuros_generic.h \
neuros_rtc.h leds.h mcbsp.h
CLOSED_HWDRIVERS_DIR:=$(BUILD_DIR)/closed-hwdrivers
CLOSED_HWDRIVERS_TARGET_MODULE_PATH:=lib/modules/$(LINUX26_VERSION_PROBED)/drivers


# Stick header files & modules all into a tarball
$(DL_DIR)/$(CLOSED_HWDRIVERS_SOURCE): 
	@$(call MESSAGE,"closed-hwdrivers - Downloading files")
	# Get header files individually via http
	cd $(DL_DIR) && \
	for i in $(CLOSED_HWDRIVERS_MODULE_FILES); \
	do \
		$(WGET) -P $(DL_DIR) $(CLOSED_HWDRIVERS_MODULE_SITE)/$$i; \
	done

	# Get modules individually via http
	cd $(DL_DIR) && \
	for i in $(CLOSED_HWDRIVERS_HEADER_FILES); \
	do \
		$(WGET) -P $(DL_DIR) $(CLOSED_HWDRIVERS_HEADER_SITE)/$$i; \
	done

	#combine the module checkout with header files already obtained
	cd $(DL_DIR) && \
	$(TAR) -cf $(basename $(CLOSED_HWDRIVERS_SOURCE)) $(CLOSED_HWDRIVERS_HEADER_FILES) \
		$(CLOSED_HWDRIVERS_MODULE_FILES) && \
	gzip -f $(basename $(CLOSED_HWDRIVERS_SOURCE))
	#Clean up
	cd $(DL_DIR) && rm $(CLOSED_HWDRIVERS_HEADER_FILES) $(CLOSED_HWDRIVERS_MODULE_FILES)

# Extract tarball containing modules and header files
$(CLOSED_HWDRIVERS_DIR)/.source: $(DL_DIR)/$(CLOSED_HWDRIVERS_SOURCE)
	echo $(DL_DIR)/$(CLOSED_HWDRIVERS_SOURCE)
	@$(call MESSAGE,"closed-hwdrivers - Extracting archive")
	mkdir -p $(CLOSED_HWDRIVERS_DIR)
	$(ZCAT) $(DL_DIR)/$(CLOSED_HWDRIVERS_SOURCE) | tar -C $(CLOSED_HWDRIVERS_DIR) $(TAR_OPTIONS) -
	touch $@

# Correct binary modules to accept gcc version kernel is built with
$(CLOSED_HWDRIVERS_DIR)/.configured: $(CLOSED_HWDRIVERS_DIR)/.source
	@$(call MESSAGE,"closed-hwdrivers - Patching")

	#Perform correction to all modules
	cd $(CLOSED_HWDRIVERS_DIR) && \
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

$(CLOSED_HWDRIVERS_DIR)/.installed: $(CLOSED_HWDRIVERS_DIR)/.configured
	touch $@

closed-hwdrivers: closed-media $(CLOSED_HWDRIVERS_DIR)/.installed
# Install modules to target
	@$(call MESSAGE,"closed-hwdrivers - Installing modules to target")

ifeq ($(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_NEUROSRTC),y)
	mkdir -p $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/rtc
	cp $(CLOSED_HWDRIVERS_DIR)/neuros_rtc.ko $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/rtc/
endif
ifeq ($(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_TVP5150),y)
	mkdir -p $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/chips
	cp $(CLOSED_HWDRIVERS_DIR)/tvp5150.ko $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/chips/
endif
ifeq ($(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_AIC23),y)
	mkdir -p $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/chips
	cp $(CLOSED_HWDRIVERS_DIR)/aic23.ko $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/chips/
endif
ifeq ($(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_ITFB),y)
	mkdir -p $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/video
	cp $(CLOSED_HWDRIVERS_DIR)/itfb.ko $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/video/
endif

# Install header files
	@$(call MESSAGE,"closed-hwdrivers - Installing headers to staging")
	cd $(CLOSED_HWDRIVERS_DIR) && \
	for i in $(PACKAGE_OSDRE_HWDRIVERS_H-y); \
	do \
		cp "$$i" $(STAGING_DIR)/usr/include/linux/; \
	done

	@$(call MESSAGE,"closed-hwdrivers - Installation complete")

closed-hwdrivers-source: $(DL_DIR)/$(CLOSED_HWDRIVERS_SOURCE)

closed-hwdrivers-clean: closed-hwdrivers-dirclean closed-hwdrivers-uninstall

closed-hwdrivers-uninstall:
	rm -rf $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/rtc/neuros_rtc.ko
	rm -rf $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/chips/aic23.ko
	rm -rf $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/chips/tvp5150.ko
	rm -rf $(TARGET_DIR)/$(CLOSED_HWDRIVERS_TARGET_MODULE_PATH)/video/itfb.ko

	rm -rf $(addprefix $(STAGING_DIR)/usr/include/linux/, $(CLOSED_HWDRIVERS_HEADER_FILES))

closed-hwdrivers-dirclean:
	rm -rf $(CLOSED_HWDRIVERS_DIR)


################################################################################
#
# Toplevel Makefile options
#
################################################################################

# What modules to include?
PACKAGE_OSDRE_HWDRIVERS-y:=
PACKAGE_OSDRE_HWDRIVERS-$(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_NEUROSRTC) += neuros_rtc.ko
PACKAGE_OSDRE_HWDRIVERS-$(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_TVP5150) += tvp5150.ko
PACKAGE_OSDRE_HWDRIVERS-$(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_AIC23) += aic23.ko
PACKAGE_OSDRE_HWDRIVERS-$(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_ITFB) += itfb.ko

# What header files to include?
PACKAGE_OSDRE_HWDRIVERS_H-y:=
PACKAGE_OSDRE_HWDRIVERS_H-$(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_NEUROSRTC) += neuros_ir_blaster.h \
neuros_ir.h neuros_generic.h neuros_rtc.h leds.h
PACKAGE_OSDRE_HWDRIVERS_H-$(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_AIC23) += aic23.h
PACKAGE_OSDRE_HWDRIVERS_H-$(BR2_PACKAGE_OSDRE_CLOSED_HWDRIVERS_ITFB) += itfb.h mcbsp.h


# If anything above required, add this makefile to the build process
ifneq ($(PACKAGE_OSDRE_CLOSED_HWDRIVERS-y),"")
TARGETS+=closed-hwdrivers
endif

