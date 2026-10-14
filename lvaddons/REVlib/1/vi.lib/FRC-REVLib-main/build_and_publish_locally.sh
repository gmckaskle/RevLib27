#! /bin/bash
set -e

BUILD_YEAR=2027_alpha5
VENDOR_GENERATED_DIR=./build/generated/vendordeps/*
BUILD_REPO_DIR=./build/repos
RELEASE_DIR=$BUILD_REPO_DIR/releases/com/revrobotics/frc
MOVE_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/maven/com/revrobotics/frc
VENDOR_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/vendordeps

echo "*** Building ***"
./gradlew build -PreleaseMode

echo "*** Generating maven directories ***"
./gradlew publish -PreleaseMode

echo "*** Ensuring destination maven directories exist ***"
mkdir -p $MOVE_DIR
mkdir -p $VENDOR_DIR

echo "*** Moving maven directories ***"
cp -r $RELEASE_DIR/RevLibBackendDriver $MOVE_DIR
cp -r $RELEASE_DIR/REVLib-cpp $MOVE_DIR
cp -r $RELEASE_DIR/REVLib-driver $MOVE_DIR
cp -r $RELEASE_DIR/REVLib-java $MOVE_DIR
cp -r $RELEASE_DIR/RevLibWpiBackendDriver $MOVE_DIR

echo "*** Moving vendor deps ***"
cp -r $VENDOR_GENERATED_DIR $VENDOR_DIR
