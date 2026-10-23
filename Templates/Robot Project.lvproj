<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="23008000">
	<Property Name="NI.LV.All.SaveVersion" Type="Str">23.0</Property>
	<Property Name="NI.LV.All.SourceOnly" Type="Bool">true</Property>
	<Property Name="NI.Project.Description" Type="Str"></Property>
	<Item Name="My Computer" Type="My Computer">
		<Property Name="NI.SortType" Type="Int">3</Property>
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="OpModes" Type="Folder">
			<Item Name="AutoOp Default.vi" Type="VI" URL="../AutoOp Default.vi"/>
			<Item Name="TeleOp Default.vi" Type="VI" URL="../TeleOp Default.vi"/>
			<Item Name="UtilityOp Default.vi" Type="VI" URL="../UtilityOp Default.vi"/>
		</Item>
		<Item Name="Robot Main.vi" Type="VI" URL="../Robot Main.vi"/>
		<Item Name="Declare OpModes.vi" Type="VI" URL="../Declare OpModes.vi"/>
		<Item Name="SubSystems.vi" Type="VI" URL="../SubSystems.vi"/>
		<Item Name="Handle OpModes.vi" Type="VI" URL="/&lt;vilib&gt;/MRC/MrcLib/VIs/Robot Framework/Handle OpModes.vi"/>
		<Item Name="Dependencies" Type="Dependencies"/>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
