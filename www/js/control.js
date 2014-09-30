var canEdit = false;
var canEditMarkers = false;
var canEditBounds = false;

function manualMode() {
	if (!canEdit) {
		$("#manual-holder").show();
		$("#auto-holder").hide();
		$("#status-holder").hide();
		showMarkers(markers);
		hideMarkers(bounds);
		hideRectangle();
		
		ajaxSend('updateWaypoints', markers);
	}
}

function autoMode() {
	if (!canEdit) {
		$("#manual-holder").hide();
		$("#auto-holder").show();
		$("#status-holder").hide();
		hideMarkers(markers);
		showMarkers(bounds);
		showRectangle();
		
		ajaxSend('updateWaypoints', bounds);
	}
}

function statusMode() {
	if (!canEdit) {
		$("#manual-holder").hide();
		$("#auto-holder").hide();
		$("#status-holder").show();
	}
}

function toggleEdit(data) {
	$("#manual-edit").toggleClass('btn-pressed');
	$("#auto-edit").toggleClass('btn-pressed');
	
	$("#main-vertical").toggleClass('orange-toggle');
	$("#main-side").toggleClass('orange-toggle');
	$("#main-bottom").toggleClass('orange-toggle');
	
	$("button[id*='begin']").toggleClass('btn-disabled');
	
	canEdit = !canEdit;
	
	if (canEdit) {
		$("#information").html('<span id="blinkbox" style="background-color: #FB8500;">&nbsp;&nbsp;&nbsp;&nbsp;</span> Edit mode engaged. Use the map.');
		$("#blinkbox").blink({delay:1000});
		
		$.each( data, function( index, value ){
			value.dragging.enable();
		});
	} else {
		$("#blinkbox").unblink();
		$("#information").html('');
		
		$.each( data, function( index, value ){
			value.dragging.disable();
		});
		
		ajaxSend('updateWaypoints', data);
	}
}

function toggleMarkersEdit() {
	toggleEdit(markers);
	canEditMarkers = !canEditMarkers;
	
	toggleMarkerRed(markers);
}

function toggleBoundsEdit() {
	toggleEdit(bounds);
	canEditBounds = !canEditBounds;

	toggleMarkerRed(bounds);
}