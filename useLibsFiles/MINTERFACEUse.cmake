#############################################################################
#
# $Id: VISPUse.cmake.in,v 1.2 2006/05/30 08:35:02 fspindle Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit.
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# cmake PackageConfig file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

INCLUDE(${CMAKE_ROOT}/Modules/CMakeImportBuildSettings.cmake)

# Import MINTERFACE's build settings:
#CMAKE_IMPORT_BUILD_SETTINGS(${VISP_BUILD_SETTINGS_FILE})

# Tell the compiler where to find ViSP's header files
# and the third party headers we depend on
#SET(MINTERFACE_EXTERN_INCLUDE_DIR "/usr/include")
LIST(APPEND MINTERFACE_INCLUDE_DIR)
MESSAGE("MINTERFACE_INCLUDE_DIR " ${MINTERFACE_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${MINTERFACE_INCLUDE_DIR} )

# Tell the compiler where to find ViSP's libraries
# and the third party libraries we depend on
LINK_DIRECTORIES(${MINTERFACE_LINK_DIRECTORIES})

LINK_LIBRARIES(${MINTERFACE_LIBRARIES})

