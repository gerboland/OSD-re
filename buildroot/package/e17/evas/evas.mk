################################################################################
#
# evas -- EFL EVAS library
#
################################################################################

EFL_EVAS_VERSION = 1.0.0
EFL_EVAS_SOURCE = evas-$(EFL_EVAS_VERSION).tar.gz
EFL_EVAS_SITE = http://download.enlightenment.org/releases/
EFL_EVAS_AUTORECONF = NO
EFL_EVAS_INSTALL_STAGING = YES
EFL_EVAS_INSTALL_TARGET = YES
EFL_EVAS_DEPENDENCIES = pkg-config freetype efl_eina
#EFL_EVAS_DEPENDENCIES += tiff librsvg libpng jpeg libungif

ifeq ($(BR2_PACKAGE_EFL_EVAS_EXAMPLES),y)
EFL_EVAS_CONF_OPT += --enable-build-examples
endif


$(eval $(call AUTOTARGETS,package/efl,efl_evas))
$(eval $(call AUTOTARGETS,package/efl,efl_evas,host))
