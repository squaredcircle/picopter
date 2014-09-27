<!DOCTYPE html>
<html>
	<head>
		<!-- Header. CSS includes. -->
		<meta charset="utf-8">
		<title>Copter LCARS</title>
		<link rel="icon" href="css/trek.png" />
		<meta name="viewport" content="width=device-width, initial-scale=1.0">
		<link rel="stylesheet" type="text/css" href="css/bootstrap.min.css">
		<link rel="stylesheet" type="text/css" href="css/bootstrap-theme.min.css">
		<link rel="stylesheet" type="text/css" href="css/style.css">
		
	</head>
	<body>
		<!-- Body -->
		<div id="mainwindow">
			<div id="map-canvas"></div>
		</div>
		<div id="sidepanel">
			<!--<div id="sidepanel_top">-->	
				<div id="status"></div>
				<div id="primary-button-holder">
					<button type="button" class="hexacontrol btn btn-danger btn-lg" value="allStop"><b>All Stop</b></button>
					<button type="button" class="hexacontrol btn btn-success btn-lg" value="beginWaypointTraversal"><b>Start Flight</b></button>
				</div>
				<div id="secondary-button-holder">
					<button id="drop" class="btn btn-default btn-lg" onclick="addMarker()">Add Waypoint</button>
					<button type="button" class="btn btn-default btn-lg" onclick="clearMarkers()">Reset Waypoints</button>
				</div>
				<br/>
				<div id="response"></div>
<!--			</div>
			<div id="sidepanel_bottom">
				<object type="application/x-shockwave-flash" data="jwplayer/jwplayer.flash.swf" width="100%" height="100%" bgcolor="#000000" id="video-jwplayer" name="video-jwplayer" tabindex="0">
					<param name="allowfullscreen" value="true">
					<param name="allowscriptaccess" value="always">
					<param name="seamlesstabbing" value="true">
					<param name="wmode" value="opaque">
				</object>
				<div id="video-jwplayer_aspect" style="display: none;"></div>
				<div id="video-jwplayer_jwpsrv" style="position: absolute; top: 0px; z-index: 10;"></div>
			</div>
-->		</div>
		
		
		<!-- Footer. Javascript includes. -->
		<script src="jwplayer/jwplayer.js"></script>
		<script src="js/piCameraStream.js"></script>
		<script src="js/jquery-2.1.1.min.js"></script>
		<script src="js/bootstrap.min.js"></script>
		<script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false"></script>
		<script src="js/map.js"></script>
		<script src="js/ajax.js"></script>
		

	</body>
</html>
