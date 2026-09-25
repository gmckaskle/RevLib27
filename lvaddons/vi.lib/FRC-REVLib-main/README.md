# REV Robotics RoboRIO SDK

## Overview

This repository is for the SPARK MAX software that is run on the NI roboRIO.

## Build Requirements

1. Git
2. Visual Studio Code or some other equivalent IDE
3. Gradle Build Tool
4. Java JDK/JRE

All of these, excluding Git, can be installed and configured with the [WPILib Installer](https://github.com/wpilibsuite/allwpilib/releases), which will include specific versions for building FRC robot code. Gradle needs a C++ compiler, which is included with VS Code.

## Publishing locally

To publish a new version of REVLib for use in your local WPILib installation,
open Git Bash and run `./build_and_publish_locally.sh`. You will then be able
to do offline installations and updates of REVLib in local WPILib projects.

If you want to be able to update existing installations, make sure to bump the version in `version.properties`.

## Publishing a new official release
1. Set the version number in `version.properties`. Commit and push.
   1. Wait for GitHub actions to finish building the project before continuing.
2. Create and push a tag for the version number in the form `vXXXX.X.X`.
   This will kick off a GitHub actions workflow to prepare the release.
3. Publish the draft release that was created at https://github.com/REVrobotics/REV-Software-Binaries/releases/
4. Merge the PRs to the following repositories:
   1. [Software-Update-Metadata](https://github.com/REVrobotics/Software-Update-Metadata)
   2. [codedocs.revrobotics.com](https://github.com/REVrobotics/codedocs.revrobotics.com)
   3. [maven.revrobotics.com](https://github.com/REVrobotics/maven.revrobotics.com)

## Publishing to private Maven Repo

To publish a new version of REVLib for use in Hardware Client, or other internal
projects, run `./gradlew publishAllToPrivate -Puse-private-repo`. This will make
the version available for use in
`artifactregistry://us-central1-maven.pkg.dev/rev-hardware-client/private-01977979-bcc8-78b0-b419-93d4efd6d4d0-revrobotics`
and its mirror,
`artifactregistry://us-central1-maven.pkg.dev/rev-hardware-client/public-01977979-bcc8-78b0-b419-93d4efd6d4d0-revrobotics`.
Note that you cannot re-upload the same version.

## State philosophy
It is OK for the high-level API classes to contain immutable state, but all mutable state should be
tracked in the driver layer. The only exceptions to this must only related to the high-level APIs,
and not be relevant to the actual functioning of the SPARK MAX.

## Adding new JNI methods
1. Define the JNI method signature in the appropriate Java file.
2. Build the project (which will fail with the error "Found a definition that does not have a matching symbol")
3. Copy the newly-generated C++ function prototype located in the corresponding header in
   `build/generated/sources/headers/java/main`, and use it as a starting point for the function definition in
   the appropriate C++ file.

## Changelog

The SDK Changelog can be viewed with [Changelog.md](Changelog.md).
