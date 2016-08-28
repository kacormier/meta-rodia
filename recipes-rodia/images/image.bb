#######################################################
#                                                     #
#     Rodia Development Image Recipe                  #
#                                                     #
#######################################################

# Baseline on core-image-minimal 
require ../sources/poky/meta/recipes-core/images/core-image-minimal.bb

# Provide debug features
IMAGE_FEATURES += "ssh-server-dropbear"

# Add miscellaneous commands
CORE_IMAGE_EXTRA_INSTALL += " \
    sysstat \
    openssh-sftp-server \
"

# Add package management
# RPM is default in local.conf
IMAGE_FEATURES += "package-management"

# As per Yocto RM use the IMAGE_INSTALL_append operator
# instead of IMAGE_INSTALL +=  to avoid ordering issues.
# Do not forget the leading space with the _append operator.

# Add general components
IMAGE_INSTALL_append = " simdrv"
IMAGE_INSTALL_append = " usbutils"

