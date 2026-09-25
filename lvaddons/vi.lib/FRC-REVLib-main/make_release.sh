#! /bin/bash

# TODO: Use maven tool or similar to modify .xml files for
# release versions that we want

RELEASE_DIR=./build/repos/releases/com/revrobotics/frc
BUILD_REPO_DIR=./build/repos/releases
STAGING_DIR=./build/release-staging
VENDOR_JSON=C:/Users/Public/wpilib/2023/vendordeps/REVLib.json
LOCAL_RELEASE_DIR=./build/release

NAME_PREFIX=SPARK-MAX-SDK-v
NAME_POSTFIX=.zip

WEB_RELEASE_FOLDER=RevRobotics.com
MAVEN_STRUCTURE=maven/com/revrobotics

## Program Start

if [ "$1" == "clean" ]; then
    rm -rf ${LOCAL_RELEASE_DIR}
    exit
fi

VERSION="$(cat ${BUILD_REPO_DIR}/com/revrobotics/frc/REVLib-cpp/maven-metadata.xml | grep -oP '(?<=<release>).*?(?=</release>)')"

if [ $? -ne 0 ]; then
    echo "Failed to get version, did you publish with ./gradlew publish?"
    exit
else
    echo "Building release: v${VERSION}"
fi

FULL_ZIP_NAME=${NAME_PREFIX}${VERSION}${NAME_POSTFIX}

# All releases go in a release folder
mkdir ${LOCAL_RELEASE_DIR}

#######################
# 1) Create zip file  #
#######################
echo "Moving files to staging..."

mkdir -p "${STAGING_DIR}/zip"
mkdir -p "${STAGING_DIR}/zip/vendordeps"

cp -r "${BUILD_REPO_DIR}/" "${STAGING_DIR}/zip/maven"
cp ${VENDOR_JSON} "${STAGING_DIR}/zip/vendordeps/"

echo "Creating ${FULL_ZIP_NAME}"

# Need something that ships with git-bash (Windows) by default
perl -e "use IO::Compress::Zip qw(:all); chdir('${STAGING_DIR}/zip');" \
     -e 'use File::Find; sub get_all_files { my ($dir) = "@_"; my @files; find(sub {if(-f $_){push @files, $File::Find::name }}, $dir); return @files}' \
     -e "my @files = get_all_files('.');" \
     -e 'foreach my $file(@files) { $file = substr $file, 2; };' \
     -e 'zip [ @files ] => "$ARGV[0]" or die "Cannot create zip file: $ZipError";' ${FULL_ZIP_NAME}

cp ${STAGING_DIR}/zip/${FULL_ZIP_NAME} ${LOCAL_RELEASE_DIR}

echo "Removing staged files..."
rm -rf ${STAGING_DIR}


#####################################
# 2) Make release folder that is    #
#    uploaded to RevRobotics.com    #
#####################################
echo "Copying files for web release..."

mkdir -p ${LOCAL_RELEASE_DIR}/${WEB_RELEASE_FOLDER}/${MAVEN_STRUCTURE}
mkdir -p ${LOCAL_RELEASE_DIR}/${WEB_RELEASE_FOLDER}/vendordeps

cp -r ${RELEASE_DIR} ${LOCAL_RELEASE_DIR}/${WEB_RELEASE_FOLDER}/${MAVEN_STRUCTURE}
cp ${VENDOR_JSON} ${LOCAL_RELEASE_DIR}/${WEB_RELEASE_FOLDER}/vendordeps
