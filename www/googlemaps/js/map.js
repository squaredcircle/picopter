var uwa = new google.maps.LatLng(-31.979839, 115.817546);
var markers = [];
var copterMarker;
var map;

function initialize() {
	var mapOptions = {
		zoom: 18,
		center: uwa,
		mapTypeId: google.maps.MapTypeId.SATELLITE,
		streetViewControl: false
	};

	map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);
	map.setTilt(0);
	addCopter();
}

function addCopter() {
	copterMarker =
		new google.maps.Marker( {
			position: uwa,
			map: map,
			draggable: false,
			animation: google.maps.Animation.DROP,
			icon: 'https://chart.googleapis.com/chart?chst=d_map_pin_icon&chld=helicopter|0099CC|000000'
		});
}

function addMarker() {
	markers.push(
		new google.maps.Marker( {
			position: map.getCenter(),
			map: map,
			draggable: true,
			animation: google.maps.Animation.DROP,
			icon: 'https://chart.googleapis.com/chart?chst=d_map_pin_letter&chld='+(markers.length+1)+'|FF776B|000000'
		} )
	);

	var thisNo = markers.length-1;	
	var thisMarker = markers[thisNo];

	if (thisNo > 1) {
		var waypointLine = new google.maps.Polyline({
			path: [markers[thisNo-1],markers[thisNo]],
			geodesic: true,
			strokeColor: '#FF0000',
			strokeOpacity: 1.0,
			strokeWeight: 2
		});

		waypointLine.setMap(map);
	}
	
	ajaxSend('addWaypoint',thisMarker.position);

	google.maps.event.addListener(thisMarker, 'dragend', function() {
		ajaxSend('updateWaypoint',thisMarker.position,thisNo);
	});
}

// Sets the map on all markers in the array.
function setAllMap(map) {
	for (var i = 0; i < markers.length; i++) {
		markers[i].setMap(map);
	}
}

// Removes the markers from the map, but keeps them in the array.
function clearMarkers() {
	setAllMap(null);
	while(markers.length > 0) {
		markers.pop();
	}
	ajaxSend('resetWaypoints');
}

google.maps.event.addDomListener(window, 'load', initialize);
