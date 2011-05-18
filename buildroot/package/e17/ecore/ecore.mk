################################################################################
#
# ecore -- EFL ECORE library
#
################################################################################

EFL_ECORE_VERSION = 1.0.0
EFL_ECORE_SOURCE = ecore-$(EFL_ECORE_VERSION).tar.gz
EFL_ECORE_SITE = http://download.enlightenment.org/releases/
EFL_ECORE_AUTORECONF = NO
EFL_ECORE_INSTALL_STAGING = YES
EFL_ECORE_INSTALL_TARGET = YES
EFL_ECORE_DEPENDENCIES = pkg-config xlib_libX11 libxcb xlib_libXrandr libglib2
EFL_ECORE_DEPENDENCIES += efl_eina efl_evas

ifeq ($(BR2_PACKAGE_EFL_ECORE_ENABLE-G-MAIN-LOOP),y)
EFL_ECORE_CONF_OPT += --enable-g-main-loop
endif

$(eval $(call AUTOTARGETS,package/efl,efl_ecore))
$(eval $(call AUTOTARGETS,package/efl,efl_ecore,host))
