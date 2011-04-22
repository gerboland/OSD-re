#############################################################
#
# upk-builder
#
#############################################################

UPK_BUILDER_VERSION=e38918cd
UPK_BUILDER_SITE=git://github.com/gerboland/osd-upk-builder.git
UPK_BUILDER_SITE_METHOD=git
UPK_BUILDER_INSTALL_STAGING = NO
UPK_BUILDER_INSTALL_TARGET = NO

HOST_UPK_BUILDER_DEPENDENCIES = host-zlib

$(eval $(call AUTOTARGETS,package,upk-builder))
$(eval $(call AUTOTARGETS,package,upk-builder,host))

