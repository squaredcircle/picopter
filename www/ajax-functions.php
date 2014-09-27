<?php
	$b = Array(false => 'false', true => 'true');

//	print "Received '" . $_POST['action'] . "'<p>";
	
	if (isset($_POST["action"])) {
		switch ($_POST['action']) {
			case "requestCoords":
				$ans = $client->requestCoords();
				print $ans->lat . "," . $ans->lon;
				break;
			case "allStop":
				$ans = $client->allStop();
				print "allStop " . $b[$ans];
				break;
			case "requestStatus":
				$ans = $client->requestStatus();
				print $ans . "\n";
				break;
			case "beginWaypointTraversal":
				$ans = $client->beginWaypointTraversal();
				print "beginWaypointTraversal " . $b[$ans] . "\n";
				break;
			case "requestNextWaypoint":
				$ans = $client->requestNextWaypoint();
				print $ans-lat . ", " . $ans->lon;
				break;
			case "addWaypoint":
				if (isset($_POST["lat"])) {
					$wp = new \picopter\coordDeg();
					$wp->lat = $_POST["lat"];
					$wp->lon = $_POST["lng"];
					$ans = $client->addWaypoint($wp);
					print "addWaypoint (" . $wp->lat . ", " . $wp->lon .") " . $b[$ans] . "\n";
				} else {
					print "addWaypoint failed.";
				}
				break;
			case "updateWaypoint":
				if (isset($_POST["lat"]) && isset($_POST["no"])) {
					$wp = new \picopter\coordDeg();
					$wp->lat = $_POST["lat"];
					$wp->lon = $_POST["lng"];
					$ans = $client->updateWaypoint($wp,$_POST["no"]);
					print "updateWaypoint " . $_POST["no"] . " (" . $wp->lat . ", " . $wp->lon .") " . $b[$ans] . "\n";
				} else {
					print "updateWaypoint failed.";
				}
				break;
		
			case "removeWaypoints":
				$ans = $client->removeWaypoints();
				print "removeWaypoints " . $b[$ans] . "\n";
				break;
			case "resetWaypoints":
				$ans = $client->resetWaypoints();
				print "resetWaypoints " . $b[$ans] . "\n";
				break;
			default:
				echo "Invalid request: '" . $_POST['action'] . "'";
		}
	} else {
		echo "No request specified";
	}
?>
