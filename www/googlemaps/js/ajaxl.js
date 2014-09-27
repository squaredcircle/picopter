function ajaxSend(action,coords,no) {
	$.ajax({
		type: "POST",
		url: "ajax-thrift.php",
		data: {
			'action': action,
			'coords': String(coords),
			'no'    : no
		},
		success: function(response) {
			$('#response').html(response);
		}
	});	
}

$('.hexacontrol').click(function(){
	var clickBtnValue = $(this).attr("value");
	if ($.isEmptyObject(markers)) {
		var markerCoords = "null";
	} else {
		var markerCoords = markers[markers.length-1].position;
	}

	ajaxSend(clickBtnValue,markerCoords);
});

(function worker() {
	$.ajax({
		type: "POST",
		url:'ajax-thrift.php',
		data: {
			'action': 'requestCoords',
		},
		success: function(data) {
			var arr = data.split(',');
			copterMarker.setLatLng([parseFloat(arr[0]),parseFloat(arr[1])]).update();
		}
	});
	$.ajax({
		type: "POST",
		url:'ajax-thrift.php',
		data: {
			'action': 'requestStatus',
		},
		success: function(data) {
			$("#status").html(data);
		},
		error: function() {
			$("#status").html("ERROR: No connection to flight control program.");
		}
	});
	setTimeout(worker, 5000);
})();
