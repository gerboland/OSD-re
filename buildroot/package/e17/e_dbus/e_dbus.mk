################################################################################
#
# e_dbus -- EFL E_DBUS library
#
################################################################################

EFL_E_DBUS_VERSION = 1.0.0
EFL_E_DBUS_SOURCE = e_dbus-$(EFL_E_DBUS_VERSION).tar.gz
EFL_E_DBUS_SITE = http://download.enlightenment.org/releases/
EFL_E_DBUS_AUTORECONF = NO
EFL_E_DBUS_INSTALL_STAGING = YES
EFL_E_DBUS_INSTALL_TARGET = YES
EFL_E_DBUS_DEPENDENCIES = pkg-config dbus

$(eval $(call AUTOTARGETS,package/efl,efl_e_dbus))
$(eval $(call AUTOTARGETS,package/efl,efl_e_dbus,host))
