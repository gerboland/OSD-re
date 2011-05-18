################################################################################
#
# edje -- EFL EDJE library
#
################################################################################

EFL_EDJE_VERSION = 1.0.0
EFL_EDJE_SOURCE = edje-$(EFL_EDJE_VERSION).tar.gz
EFL_EDJE_SITE = http://download.enlightenment.org/releases/
EFL_EDJE_AUTORECONF = NO
EFL_EDJE_INSTALL_STAGING = YES
EFL_EDJE_INSTALL_TARGET = YES
EFL_EDJE_DEPENDENCIES = pkg-config fontconfig freetype lua 
EFL_EDJE_DEPENDENCIES += efl_eina efl_eet efl_embryo efl_ecore

ifeq ($(BR2_PACKAGE_EFL_EDJE_AMALGAMATION),y)
EFL_EDJE_CONF_OPT += --enable-amalgamation
endif


$(eval $(call AUTOTARGETS,package/efl,efl_edje))
$(eval $(call AUTOTARGETS,package/efl,efl_edje,host))
