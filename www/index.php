<!DOCTYPE html>
<html>
	<head>
		<!-- Header. CSS includes. -->
		<meta charset="utf-8">
		<title>Copter LCARS</title>
		<link rel="stylesheet" type="text/css" href="css/bootstrap.min.css">
		<link rel="stylesheet" type="text/css" href="css/bootstrap-theme.min.css">
		<link rel="stylesheet" type="text/css" href="css/style.css">
	</head>
	<body>
		<!-- Body -->
		<div id="mainwindow">
			<object type="application/x-shockwave-flash" data="jwplayer/jwplayer.flash.swf" width="100%" height="100%" bgcolor="#000000" id="video-jwplayer" name="video-jwplayer" tabindex="0">
					<param name="allowfullscreen" value="true">
					<param name="allowscriptaccess" value="always">
					<param name="seamlesstabbing" value="true">
					<param name="wmode" value="opaque">
			</object>
			<div id="video-jwplayer_aspect" style="display: none;"></div>
			<div id="video-jwplayer_jwpsrv" style="position: absolute; top: 0px; z-index: 10;"></div>
		
		</div>
		
		
		<div id="sidepanel">
			<div id="sidepanel_top">
				<div id="panel" style="margin-left: -52px">
					<button id="drop" onclick="addMarker()">Drop Marker</button>
				</div>
				<div id="map-canvas"></div>
			</div>
			<div id="sidepanel_bottom">
				<h1><center>UWA Hexacopter Group</center></h1>
				<br />
				<div id="gps_coords" style="font-family: Courier; font-size:30px;">
				</div>
			</div>
		</div>
		
		
		<!-- Footer. Javascript includes. -->
		<script src="jwplayer/jwplayer.js"></script>
		<script src="js/piCameraStream.js"></script>
		<script src="js/jquery-2.1.1.min.js"></script>
		<script src="js/bootstrap.min.js"></script>
		<script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false"></script>
		<script src="js/map.js"></script>
		
		<script>
			(function worker() {
				$.ajax({
						url:'ajax.php',
						success: function(data) {
							$('#gps_coords').html(data);
						},
						complete: function() {
							$.ajax({
								url:'ajax_map.php',
								success: function(data) {
									var arr = data.split(',');
									var newLatLng = new google.maps.LatLng(parseFloat(arr[0]),parseFloat(arr[1]));
									markers[0].setPosition(newLatLng);
								}
							});
							setTimeout(worker, 1000);
						}
					});
				})();
		</script>
	</body>
</html>
