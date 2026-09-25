<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="25008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="Menus" Type="Folder" URL="../Menus">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="SubVI" Type="Folder" URL="../SubVI">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="Typedef" Type="Folder" URL="../Typedef">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="VI Scripts" Type="Folder" URL="../VI Scripts">
			<Property Name="NI.DISK" Type="Bool">true</Property>
		</Item>
		<Item Name="License.rtf" Type="Document" URL="../License.rtf"/>
		<Item Name="Dependencies" Type="Dependencies"/>
		<Item Name="Build Specifications" Type="Build">
			<Item Name="REVLib Package" Type="{E661DAE2-7517-431F-AC41-30807A3BDA38}">
				<Property Name="NIPKG_addToFeed" Type="Bool">true</Property>
				<Property Name="NIPKG_allDependenciesToFeed" Type="Bool">false</Property>
				<Property Name="NIPKG_allDependenciesToSystemLink" Type="Bool">false</Property>
				<Property Name="NIPKG_certificates" Type="Bool">false</Property>
				<Property Name="NIPKG_createInstaller" Type="Bool">true</Property>
				<Property Name="NIPKG_feedLocation" Type="Path">/C/Users/Public/Documents/REV_NIPM</Property>
				<Property Name="NIPKG_installerArtifacts" Type="Str">Install.exe|InstallCHS.dll|InstallDEU.dll|InstallFRA.dll|InstallJPN.dll|InstallKOR.dll|bin|feeds|pool|system-packages
</Property>
				<Property Name="NIPKG_installerBuiltBefore" Type="Bool">true</Property>
				<Property Name="NIPKG_installerDestination" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/builds/Installer</Property>
				<Property Name="NIPKG_lastBuiltPackage" Type="Str">revlib_2026.0.1-0_windows_all.nipkg</Property>
				<Property Name="NIPKG_license" Type="Ref"></Property>
				<Property Name="NIPKG_packageVersion" Type="Bool">false</Property>
				<Property Name="NIPKG_releaseNotes" Type="Str">- [SPARK] Adds support for new feedforward parameters: `kV` (formerly `kF`), `kA`, `kS`, `kG`, `kCos`, and `kCosRatio`
- [SPARK] Renames MAXMotion parameters to be more descriptive (`kMaxVelocity` -&gt; `kCruiseVelocity`, `kAllowedClosedLoopError` -&gt; `kAllowedProfileError`)
- [SPARK] Adds support for new MAXMotion status signals: `MAXMotionSetpointPosition` and `MAXMotionSetpointVelocity`
- [SPARK] Adds support for closed loop status signals: `isAtSetpoint`, `setpoint`, and `selectedClosedLoopSlot`
- [SPARK] Adds the ability to set allowed closed loop error when using regular PID control</Property>
				<Property Name="NIPKG_storeProduct" Type="Bool">false</Property>
				<Property Name="NIPKG_VisibleForRuntimeDeployment" Type="Bool">false</Property>
				<Property Name="PKG_actions.Count" Type="Int">0</Property>
				<Property Name="PKG_autoIncrementBuild" Type="Bool">false</Property>
				<Property Name="PKG_autoSelectDeps" Type="Bool">true</Property>
				<Property Name="PKG_buildNumber" Type="Int">0</Property>
				<Property Name="PKG_buildSpecName" Type="Str">REVLib Package</Property>
				<Property Name="PKG_dependencies.Count" Type="Int">2</Property>
				<Property Name="PKG_dependencies[0].Enhanced" Type="Bool">false</Property>
				<Property Name="PKG_dependencies[0].MaxVersion" Type="Str"></Property>
				<Property Name="PKG_dependencies[0].MaxVersionInclusive" Type="Bool">false</Property>
				<Property Name="PKG_dependencies[0].MinVersion" Type="Str">25.3.3.49167-0+f15</Property>
				<Property Name="PKG_dependencies[0].MinVersionType" Type="Str">Inclusive</Property>
				<Property Name="PKG_dependencies[0].NIPKG.DisplayName" Type="Str">LabVIEW Runtime (32-bit)</Property>
				<Property Name="PKG_dependencies[0].Package.Name" Type="Str">ni-labview-2025-runtime-engine-x86</Property>
				<Property Name="PKG_dependencies[0].Package.Section" Type="Str">Programming Environments</Property>
				<Property Name="PKG_dependencies[0].Package.Synopsis" Type="Str">The LabVIEW Runtime is a software add-on that enables engineers to run executables on a nondevelopment machine.</Property>
				<Property Name="PKG_dependencies[0].Relationship" Type="Str">Required Dependency</Property>
				<Property Name="PKG_dependencies[0].Type" Type="Str">NIPKG</Property>
				<Property Name="PKG_dependencies[1].Enhanced" Type="Bool">false</Property>
				<Property Name="PKG_dependencies[1].MaxVersion" Type="Str"></Property>
				<Property Name="PKG_dependencies[1].MaxVersionInclusive" Type="Bool">false</Property>
				<Property Name="PKG_dependencies[1].MinVersion" Type="Str">25.5.0.49251-0+f99</Property>
				<Property Name="PKG_dependencies[1].MinVersionType" Type="Str">Inclusive</Property>
				<Property Name="PKG_dependencies[1].NIPKG.DisplayName" Type="Str">NI CompactRIO Driver</Property>
				<Property Name="PKG_dependencies[1].Package.Name" Type="Str">ni-compactrio-runtime</Property>
				<Property Name="PKG_dependencies[1].Package.Section" Type="Str">Drivers</Property>
				<Property Name="PKG_dependencies[1].Package.Synopsis" Type="Str">Runtime driver support for CompactRIO Reconfigurable Embedded Targets. This also includes support for sbRIO and MXI-Express Targets.</Property>
				<Property Name="PKG_dependencies[1].Relationship" Type="Str">Required Dependency</Property>
				<Property Name="PKG_dependencies[1].Type" Type="Str">NIPKG</Property>
				<Property Name="PKG_description" Type="Str">LabVIEW files for REVLib for use in the FIRST Robotics Competition</Property>
				<Property Name="PKG_destinations.Count" Type="Int">6</Property>
				<Property Name="PKG_destinations[0].ID" Type="Str">{094AEE7F-DAEF-4226-8F59-6B8C8420BFD5}</Property>
				<Property Name="PKG_destinations[0].Subdir.Directory" Type="Str">National Instruments</Property>
				<Property Name="PKG_destinations[0].Subdir.Parent" Type="Str">root_5</Property>
				<Property Name="PKG_destinations[0].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[1].ID" Type="Str">{1FB827AB-93BE-4149-8393-C6F0B9A2B673}</Property>
				<Property Name="PKG_destinations[1].Subdir.Directory" Type="Str">ThirdParty</Property>
				<Property Name="PKG_destinations[1].Subdir.Parent" Type="Str">{F744EE47-B0ED-4480-95ED-F0FD7DBD8A98}</Property>
				<Property Name="PKG_destinations[1].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[2].ID" Type="Str">{441049F1-CC33-4B58-B3D0-F00C61F3F49F}</Property>
				<Property Name="PKG_destinations[2].Subdir.Directory" Type="Str">Rock Robotics</Property>
				<Property Name="PKG_destinations[2].Subdir.Parent" Type="Str">{71B83093-3EFD-4EE2-B5CC-62CFE3815242}</Property>
				<Property Name="PKG_destinations[2].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[3].ID" Type="Str">{71B83093-3EFD-4EE2-B5CC-62CFE3815242}</Property>
				<Property Name="PKG_destinations[3].Subdir.Directory" Type="Str">vi.lib</Property>
				<Property Name="PKG_destinations[3].Subdir.Parent" Type="Str">{EABFB75F-1169-43D4-8253-AAF81607B88A}</Property>
				<Property Name="PKG_destinations[3].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[4].ID" Type="Str">{EABFB75F-1169-43D4-8253-AAF81607B88A}</Property>
				<Property Name="PKG_destinations[4].Subdir.Directory" Type="Str">LabVIEW 2025</Property>
				<Property Name="PKG_destinations[4].Subdir.Parent" Type="Str">{094AEE7F-DAEF-4226-8F59-6B8C8420BFD5}</Property>
				<Property Name="PKG_destinations[4].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_destinations[5].ID" Type="Str">{F744EE47-B0ED-4480-95ED-F0FD7DBD8A98}</Property>
				<Property Name="PKG_destinations[5].Subdir.Directory" Type="Str">WPI</Property>
				<Property Name="PKG_destinations[5].Subdir.Parent" Type="Str">{441049F1-CC33-4B58-B3D0-F00C61F3F49F}</Property>
				<Property Name="PKG_destinations[5].Type" Type="Str">Subdir</Property>
				<Property Name="PKG_displayName" Type="Str">REVLib</Property>
				<Property Name="PKG_displayVersion" Type="Str">2026.0.1</Property>
				<Property Name="PKG_feedDescription" Type="Str"></Property>
				<Property Name="PKG_feedName" Type="Str"></Property>
				<Property Name="PKG_homepage" Type="Str">www.revrobotics.com</Property>
				<Property Name="PKG_hostname" Type="Str"></Property>
				<Property Name="PKG_maintainer" Type="Str">REV Robotics &lt;support@revrobotics.com&gt;</Property>
				<Property Name="PKG_output" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/builds</Property>
				<Property Name="PKG_packageName" Type="Str">revlib</Property>
				<Property Name="PKG_publishToSystemLink" Type="Bool">false</Property>
				<Property Name="PKG_section" Type="Str">Add-Ons</Property>
				<Property Name="PKG_shortcuts.Count" Type="Int">0</Property>
				<Property Name="PKG_sources.Count" Type="Int">1</Property>
				<Property Name="PKG_sources[0].Destination" Type="Str">{1FB827AB-93BE-4149-8393-C6F0B9A2B673}</Property>
				<Property Name="PKG_sources[0].ID" Type="Ref">/My Computer/Build Specifications/WPILib Third-Party</Property>
				<Property Name="PKG_sources[0].Type" Type="Str">Build</Property>
				<Property Name="PKG_synopsis" Type="Str">REVLib LabVIEW API</Property>
				<Property Name="PKG_version" Type="Str">2026.0.1</Property>
			</Item>
			<Item Name="WPILib Third-Party" Type="Source Distribution">
				<Property Name="Bld_buildCacheID" Type="Str">{1B03D4C7-8D54-4EDE-B827-0673612478F4}</Property>
				<Property Name="Bld_buildSpecName" Type="Str">WPILib Third-Party</Property>
				<Property Name="Bld_excludedDirectory[0]" Type="Path">vi.lib</Property>
				<Property Name="Bld_excludedDirectory[0].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectory[1]" Type="Path">resource/objmgr</Property>
				<Property Name="Bld_excludedDirectory[1].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectory[2]" Type="Path">/C/ProgramData/National Instruments/InstCache/23.0</Property>
				<Property Name="Bld_excludedDirectory[3]" Type="Path">/C/Users/Jan-Felix Abellera/Documents/LabVIEW Data/2023(32-bit)/ExtraVILib</Property>
				<Property Name="Bld_excludedDirectory[4]" Type="Path">instr.lib</Property>
				<Property Name="Bld_excludedDirectory[4].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectory[5]" Type="Path">user.lib</Property>
				<Property Name="Bld_excludedDirectory[5].pathType" Type="Str">relativeToAppDir</Property>
				<Property Name="Bld_excludedDirectoryCount" Type="Int">6</Property>
				<Property Name="Bld_localDestDir" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty</Property>
				<Property Name="Bld_previewCacheID" Type="Str">{D5BBF036-E480-4B71-B45A-8C3F2620F178}</Property>
				<Property Name="Bld_removeVIObj" Type="Int">1</Property>
				<Property Name="Bld_version.major" Type="Int">2026</Property>
				<Property Name="Bld_version.patch" Type="Int">1</Property>
				<Property Name="Destination[0].destName" Type="Str">Destination Directory</Property>
				<Property Name="Destination[0].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty</Property>
				<Property Name="Destination[0].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[1].destName" Type="Str">Support Directory</Property>
				<Property Name="Destination[1].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/data</Property>
				<Property Name="Destination[1].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[2].destName" Type="Str">Public</Property>
				<Property Name="Destination[2].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/SubVI/Public</Property>
				<Property Name="Destination[2].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[3].destName" Type="Str">Private</Property>
				<Property Name="Destination[3].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/SubVI/Private</Property>
				<Property Name="Destination[3].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[4].destName" Type="Str">Typedef</Property>
				<Property Name="Destination[4].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/Typedef</Property>
				<Property Name="Destination[4].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[5].destName" Type="Str">Daemon</Property>
				<Property Name="Destination[5].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/SubVI/Private/Daemon</Property>
				<Property Name="Destination[5].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[6].destName" Type="Str">Examples</Property>
				<Property Name="Destination[6].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/SubVI/Examples</Property>
				<Property Name="Destination[6].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[7].destName" Type="Str">Menu</Property>
				<Property Name="Destination[7].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty</Property>
				<Property Name="Destination[7].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[7].preserveHierarchy" Type="Bool">true</Property>
				<Property Name="Destination[8].destName" Type="Str">REV Root</Property>
				<Property Name="Destination[8].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME</Property>
				<Property Name="Destination[8].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="Destination[9].destName" Type="Str">Deprecated</Property>
				<Property Name="Destination[9].path" Type="Path">/C/Users/Public/Documents/REVLib-LabVIEW/WPILib/ThirdParty/REV Robotics/NI_AB_PROJECTNAME/SubVI/Deprecated</Property>
				<Property Name="Destination[9].path.type" Type="Str">&lt;none&gt;</Property>
				<Property Name="DestinationCount" Type="Int">10</Property>
				<Property Name="Source[0].itemID" Type="Str">{853FDCCF-F637-4A1E-A953-9044D371CCDE}</Property>
				<Property Name="Source[0].type" Type="Str">Container</Property>
				<Property Name="Source[1].destinationIndex" Type="Int">8</Property>
				<Property Name="Source[1].itemID" Type="Ref">/My Computer/License.rtf</Property>
				<Property Name="Source[1].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[2].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[2].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[2].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[2].destinationIndex" Type="Int">7</Property>
				<Property Name="Source[2].itemID" Type="Ref">/My Computer/Menus</Property>
				<Property Name="Source[2].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[2].type" Type="Str">Container</Property>
				<Property Name="Source[3].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[3].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[3].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[3].destinationIndex" Type="Int">4</Property>
				<Property Name="Source[3].itemID" Type="Ref">/My Computer/Typedef</Property>
				<Property Name="Source[3].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[3].type" Type="Str">Container</Property>
				<Property Name="Source[4].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[4].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[4].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[4].itemID" Type="Ref">/My Computer/SubVI/Debug</Property>
				<Property Name="Source[4].sourceInclusion" Type="Str">Exclude</Property>
				<Property Name="Source[4].type" Type="Str">Container</Property>
				<Property Name="Source[5].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[5].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[5].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[5].destinationIndex" Type="Int">3</Property>
				<Property Name="Source[5].itemID" Type="Ref">/My Computer/SubVI/Private</Property>
				<Property Name="Source[5].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[5].type" Type="Str">Container</Property>
				<Property Name="Source[6].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[6].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[6].destinationIndex" Type="Int">0</Property>
				<Property Name="Source[6].itemID" Type="Ref">/My Computer/VI Scripts</Property>
				<Property Name="Source[6].sourceInclusion" Type="Str">Exclude</Property>
				<Property Name="Source[6].type" Type="Str">Container</Property>
				<Property Name="Source[7].Container.applyDestination" Type="Bool">true</Property>
				<Property Name="Source[7].Container.applyInclusion" Type="Bool">true</Property>
				<Property Name="Source[7].Container.depDestIndex" Type="Int">0</Property>
				<Property Name="Source[7].destinationIndex" Type="Int">2</Property>
				<Property Name="Source[7].itemID" Type="Ref">/My Computer/SubVI/Public</Property>
				<Property Name="Source[7].sourceInclusion" Type="Str">Include</Property>
				<Property Name="Source[7].type" Type="Str">Container</Property>
				<Property Name="SourceCount" Type="Int">8</Property>
			</Item>
		</Item>
	</Item>
</Project>
