var uwa = [-31.979839, 115.817546];
var markers = [];
var copterMarker;
var map;

function initialise() {
	map = L.map('map-canvas').setView(uwa, 16);

	L.tileLayer('http://192.168.1.13/test/leaflet/tiles/{z}/{x}/{y}.jpg', {
		maxZoom: 18,
		minZoom: 16
	}).addTo(map);
	
	addCopter();
}

function addCopter() {
	copterMarker =
		new L.marker( uwa, {
			draggable: false,
			icon: '/css/heli.png'
		}).addTo(map);
}

function addMarker() {
	markers.push(
		new L.marker( map.getCenter(), {
			draggable: true,
			icon:	new L.NumberedDivIcon({number: (markers.length+1)})
		} ).addTo(map)
	);

	var thisNo = markers.length-1;	
	var thisMarker = markers[thisNo];
/*
	if (thisNo > 1) {
		var waypointLine = new google.maps.Polyline({
			path: [markers[thisNo-1],markers[thisNo]],
			geodesic: true,
			strokeColor: '#FF0000',
			strokeOpacity: 1.0,
			strokeWeight: 2
		});

		waypointLine.setMap(map);
	}*/
	
	ajaxSend('addWaypoint',thisMarker.position);

	thisMarker.on('dragend', function(event) {
		ajaxSend('updateWaypoint',thisMarker.position,thisNo);
	});
}


// Removes the markers from the map, but keeps them in the array.
function clearMarkers() {
	for (var i = 0; i < markers.length; i++) {
		map.removeLayer(markers[i]);
	}
	
	while(markers.length > 0) {
		markers.pop();
	}
	ajaxSend('resetWaypoints');
}

$(function() {
    initialise();
});



		
		
	/*	
		L.tileLayer('https://{s}.tiles.mapbox.com/v3/{id}/{z}/{x}/{y}.png', {
			maxZoom: 18,
			attribution: 'Map data &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors, ' +
				'<a href="http://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, ' +
				'Imagery © <a href="http://mapbox.com">Mapbox</a>',
			id: 'examples.map-i875mjb7'
		}).addTo(map);


		L.marker([-31.979839, 115.817546]).addTo(map)
			.bindPopup("<b>Hello world!</b><br />I am a popup.").openPopup();



		var popup = L.popup();

		function onMapClick(e) {
			popup
				.setLatLng(e.latlng)
				.setContent("You clicked the map at " + e.latlng.toString())
				.openOn(map);
		}

		map.on('click', onMapClick);

*/