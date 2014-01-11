// Define our global variables
var GoogleMap     = null;
var Planes        = {};
var PlanesOnMap   = 0;
var PlanesOnTable = 0;
var PlanesToReap  = 0;
var SelectedPlane = null;
var SpecialSquawk = false;
var MetarICAO     = null;
var MetarReset    = true;
var AntennaData     = {};
var AntennaDataPath = null;

// Track all the plane data in a massive array for the html table...
var data_array = [];
var data_table = null;

// These might go away...
var iSortCol=-1;
var bSortASC=true;
var bDefaultSortASC=true;
var iDefaultSortCol=3;

// Get current map settings
CenterLat = Number(localStorage['CenterLat']) || CONST_CENTERLAT;
CenterLon = Number(localStorage['CenterLon']) || CONST_CENTERLON;
ZoomLvl   = Number(localStorage['ZoomLvl']) || CONST_ZOOMLVL;

// Set ajax data type. Datatype 'jsonp' is needed when using json from different port or server.
// This way data type can be set from config.js or untracked[...].js files.
if (typeof(ajaxDataType) === 'undefined') {
    ajaxDataType = 'json';
}

var iPlanesTrackable = 0;
var iPlanesTable = 0;
var iPlanesTotal = 0;

function fetchData() {
    $.ajax({
	    url: CONST_JSON,
	    dataType: ajaxDataType,
	    success: function(data) {
		    iPlanesTotal = 0
		    SpecialSquawk = false;
		
		    // Loop through all the planes in the data packet
		    for (var j=0; j < data.length; j++) {
			    // Do we already have this plane object in Planes?
			    // If not make it.
			    if (Planes[data[j].hex]) {
				    var plane = Planes[data[j].hex];
			    } else {
				    var plane = jQuery.extend(true, {}, planeObject);
			    }
			
			    /* For special squawk tests
				if (data[j].hex == 'xxxxxx') {
                    data[j].squawk = '7700';
		        }*/
                iPlanesTotal = j;

                // Set SpecialSquawk-value
                if (data[j].squawk == '7500' || data[j].squawk == '7600' || data[j].squawk == '7700') {
                    SpecialSquawk = true;
                }

			    // Call the function update
			    plane.funcUpdateData(data[j]);
			
			    // Copy the plane into Planes
			    Planes[plane.icao] = plane;
		    }
	    }
    });
}

// Initalizes the map and starts up our timers to call various functions
function initialize() {
	// Make a list of all the available map IDs
	var mapTypeIds = [];
	for(var type in google.maps.MapTypeId) {
		mapTypeIds.push(google.maps.MapTypeId[type]);
	}
	// Push OSM on to the end
	mapTypeIds.push("OSM");
	mapTypeIds.push("dark_map");

	// Styled Map to outline airports and highways
	var styles = [
		{
			"featureType": "administrative",
			"stylers": [
				{ "visibility": "off" }
			]
		},{
			"featureType": "landscape",
			"stylers": [
				{ "visibility": "off" }
			]
		},{
			"featureType": "poi",
			"stylers": [
				{ "visibility": "off" }
			]
		},{
			"featureType": "road",
			"stylers": [
				{ "visibility": "off" }
			]
		},{
			"featureType": "transit",
			"stylers": [
				{ "visibility": "off" }
			]
		},{
			"featureType": "landscape",
			"stylers": [
				{ "visibility": "on" },
				{ "weight": 8 },
				{ "color": "#0F0F0F" }
			]
		},{
			"featureType": "water",
			"stylers": [
			{ "lightness": -74 }
			]
		},{
			"featureType": "transit.station.airport",
			"stylers": [
				{ "visibility": "on" },
				{ "weight": 8 },
				{ "invert_lightness": true },
				{ "lightness": 27 }
			]
		},{
			"featureType": "road.highway",
			"stylers": [
				{ "visibility": "simplified" },
				{ "invert_lightness": true },
				{ "gamma": 0.3 }
			]
		},{
			"featureType": "road",
			"elementType": "labels",
			"stylers": [
				{ "visibility": "off" }
			]
		}
	]

	// Add our styled map
	var styledMap = new google.maps.StyledMapType(styles, {name: "Dark Map"});

	// Define the Google Map
	var mapOptions = {
		center: new google.maps.LatLng(CenterLat, CenterLon),
		zoom: ZoomLvl,
		mapTypeId: google.maps.MapTypeId.ROADMAP,
		mapTypeControlOptions: {
			mapTypeIds: mapTypeIds,
		}
	};

	GoogleMap = new google.maps.Map(document.getElementById("map_canvas"), mapOptions);

	//Define OSM map type pointing at the OpenStreetMap tile server
	GoogleMap.mapTypes.set("OSM", new google.maps.ImageMapType({
		getTileUrl: function(coord, zoom) {
			return "http://tile.openstreetmap.org/" + zoom + "/" + coord.x + "/" + coord.y + ".png";
		},
		tileSize: new google.maps.Size(256, 256),
		name: "OpenStreetMap",
		maxZoom: 18
	}));

	GoogleMap.mapTypes.set("dark_map", styledMap);
	
    // Listeners for newly created Map
    google.maps.event.addListener(GoogleMap, 'center_changed', function() {
        localStorage['CenterLat'] = GoogleMap.getCenter().lat();
        localStorage['CenterLon'] = GoogleMap.getCenter().lng();
    });

    google.maps.event.addListener(GoogleMap, 'zoom_changed', function() {
        localStorage['ZoomLvl']  = GoogleMap.getZoom();
    });
	
	// Add home marker if requested
	if (SiteShow && (typeof SiteLat !==  'undefined' || typeof SiteLon !==  'undefined')) {
	    var siteMarker  = new google.maps.LatLng(SiteLat, SiteLon);
	    
	    var markerImage = {
	        url:    'http://maps.google.com/mapfiles/kml/pal4/icon57.png',
            size:   new google.maps.Size(32, 32),   // Image size
            origin: new google.maps.Point(0, 0),    // Origin point of image
            anchor: new google.maps.Point(16, 16)   // Position where marker should point
        };
             
	    var marker = new google.maps.Marker({
            position: siteMarker,
            map: GoogleMap,
            icon: markerImage,
            title: 'My Radar Site',
            zIndex: -99999
        });
        
        if (SiteCircles) {
            for (var i=0;i<SiteCirclesDistances.length;i++) {
              drawCircle(marker, SiteCirclesDistances[i]); // in meters
            }
        }
        
        if (AntennaDataCollect) {
            if (localStorage.getObject('AntennaData')) {
                AntennaData = localStorage.getObject('AntennaData');
            }
        }
        
        if (AntennaDataShow) {
            // Get AntennaData from localStorage
            if (localStorage['AntennaData']) {AntennaData = localStorage.getObject('AntennaData');}
            
            // Load data from file
            length = Object.size(AntennaData);
            if (length < 270) {
                jQuery.ajaxSetup({async:false});
                $.get('antennaBaseCoverage.txt',  function(data) {
                    if (data.indexOf('Error') == -1) { // no errors
                        localStorage['AntennaData'] = data;
                    }
                });
                jQuery.ajaxSetup({async:true});
            }
            
            if (localStorage['AntennaData']) {
                AntennaData = localStorage.getObject('AntennaData');
                drawAntennaData(siteMarker);
            }
        }
	}
	
	// Ui buttons setup
	btnWidth = "100px";
	$("#resetMap").button({icons: {primary: "ui-icon-arrowrefresh-1-w"}});
	$("#resetMap").width(btnWidth);
	$("#resetMap").css("margin-bottom", "3px");
	$("#resetMap").button().focus(function() {
           $(this).button("widget").removeClass("ui-state-focus");
    });
	
	$("#optionsModal").button({icons: {primary: "ui-icon-gear"}});
	$("#optionsModal").width(btnWidth);
    $("#optionsModal").button().focus(function() {
        $(this).button("widget").removeClass("ui-state-focus");
    });
	
	// Load up our options page
	optionsInitalize();

	// Did our crafty user need some setup?
	extendedInitalize();
	
	// Setup our timer to poll from the server.
	window.setInterval(function() {
		fetchData();
        updateTableOfPlanes();
		refreshSelected();
		reaper();
		extendedPulse();
	}, 1000);
	
	// Refresh metar now and then only once every 5 minutes.
	if (MetarIcaoCode && MetarIcaoCode != "") {
	    getMetar();
	    $("#METAR").on("drag", function(event, ui) {
	        MetarDragged = true;
	});

    window.setInterval(function() {
            getMetar();
        }, 300000);
    }

    data_table = $('#table_of_planes').dataTable( {
        "aaData": data_array,
        "aoColumns": [
            { "sTitle": "Flag",             "sWidth": "50px",   "sClass": "col-center",     "bSortable": false, "mData": "flag" },
            { "sTitle": "Reg",              "sWidth": "50px",   "sClass": "col-left",                           "mData": "registration" },
            { "sTitle": "Operator Logo",    "sWidth": "50px",   "sClass": "col-center",     "bSortable": false, "mData": "operator_logo" },
            { "sTitle": "Silhouette",       "sWidth": "50px",   "sClass": "col-center",     "bSortable": false, "mData": "silhouette" },
            { "sTitle": "Flight",           "sWidth": "50px",   "sClass": "col-left",                           "mData": "flight" },

            { "sTitle": "Alt",              "sWidth": "50px",   "sClass": "col-right",                          "mData": "altitude"},
            { "sTitle": "Spd",              "sWidth": "50px",   "sClass": "col-right",                          "mData": "speed" },
            { "sTitle": "Trk",              "sWidth": "50px",   "sClass": "col-right",                          "mData": "track" },
            { "sTitle": "Latitude",         "sWidth": "50px",   "sClass": "col-center",                         "mData": "latitude",        "bVisible": false, },
            { "sTitle": "Longitude",        "sWidth": "50px",   "sClass": "col-center",                         "mData": "longitude",       "bVisible": false, },

            { "sTitle": "Squawk",           "sWidth": "50px",   "sClass": "col-center",                         "mData": "squawk" },
            { "sTitle": "ICAO",             "sWidth": "50px",   "sClass": "col-center",                         "mData": "icao" },
            { "sTitle": "MSGs",             "sWidth": "50px",   "sClass": "col-right",                          "mData": "messages",        "bVisible": false,},
            { "sTitle": "Seen",             "sWidth": "50px",   "sClass": "col-right",                          "mData": "seen",            "bVisible": false,},
            { "sTitle": "Country",          "sWidth": "50px",   "sClass": "col-center",                         "mData": "country",         "bVisible": false,},
            { "sTitle": "Country Short",    "sWidth": "50px",   "sClass": "col-center",                         "mData": "country_short",   "bVisible": false,},
            { "sTitle": "Type",             "sWidth": "50px",   "sClass": "col-center",                         "mData": "type",            "bVisible": false,},
            { "sTitle": "Operator",         "sWidth": "50px",   "sClass": "col-center",                         "mData": "operator",        "bVisible": false,},
        ],
        "asStripeClasses": [ "row" ],
        "bPaginate": false,
        "bLengthChange": false,
        "bFilter": false,
        "bSort": true,
        "bInfo": false,
        "bAutoWidth": false,
        "bStateSave": true,
        "bJQueryUI": true,
        "sDom": 'Rt',
    });

    $("#table_of_planes").on('click', 'tr', function(event) {
        var id = data_table.fnGetData(this);
        onClickPlanes_table(id['icao'])
        updateTableOfPlanes();
    });
}

// This looks for planes to reap out of the master Planes variable
function reaper() {
	PlanesToReap = 0;
	// When did the reaper start?
	reaptime = new Date().getTime();
	// Loop the planes
	for (var reap in Planes) {
		// Is this plane possibly reapable?
		if (Planes[reap].reapable == true) {
			// Has it not been seen for 5 minutes?
			// This way we still have it if it returns before then
			// Due to loss of signal or other reasons
			if ((reaptime - Planes[reap].updated) > 300000) {
				// Reap it.
				delete Planes[reap];
			}
			PlanesToReap++;
		}
	};
} 

// Refresh the detail window about the plane
function refreshSelected() {
    var selected = false;
	if (typeof SelectedPlane !== 'undefined' && SelectedPlane != "ICAO" && SelectedPlane != null) {
    	selected = Planes[SelectedPlane];
    }
	
	var columns = 2;
	var html = '';
	
	if (selected) {
    	html += '<table id="selectedinfo" width="100%">';
    } else {
        html += '<table id="selectedinfo" class="dim" width="100%">';
    }
	
	// Flight header line including squawk if needed
	if (selected && selected.flight == "") {
	    html += '<tr><td colspan="' + columns + '" id="selectedinfotitle"><b>N/A (' +
	        selected.icao + ')</b>';
	} else if (selected && selected.flight != "") {
	    html += '<tr><td colspan="' + columns + '" id="selectedinfotitle"><b>' +
	        selected.flight + '</b>';
	} else {
	    html += '<tr><td colspan="' + columns + '" id="selectedinfotitle"><b>DUMP1090</b>';
	}
	
	if (selected && selected.squawk == 7500) { // Lets hope we never see this... Aircraft Hijacking
		html += '&nbsp;<span class="squawk7500">&nbsp;Squawking: Aircraft Hijacking&nbsp;</span>';
	} else if (selected && selected.squawk == 7600) { // Radio Failure
		html += '&nbsp;<span class="squawk7600">&nbsp;Squawking: Radio Failure&nbsp;</span>';
	} else if (selected && selected.squawk == 7700) { // General Emergency
		html += '&nbsp;<span class="squawk7700">&nbsp;Squawking: General Emergency&nbsp;</span>';
	} else if (selected && selected.flight != '') {
	    html += '&nbsp;<a href="http://www.flightstats.com/go/FlightStatus/flightStatusByFlight.do?';
        html += 'flightNumber='+selected.flight+'" target="_blank">[FlightStats]</a>';
	}
	html += '<td></tr>';
	
	if (selected && selected.altitude != '') {
	    if (Metric) {
        	html += '<tr><td>Altitude: ' + Math.round(selected.altitude / 3.2828) + ' m</td>';
        } else {
            html += '<tr><td>Altitude: ' + selected.altitude + ' ft</td>';
        }
    } else {
        html += '<tr><td>Altitude: n/a</td>';
    }
		
	if (selected && selected.squawk != '0000') {
		html += '<td>Squawk: ' + selected.squawk + '</td></tr>';
	} else {
	    html += '<td>Squawk: n/a</td></tr>';
	}
	
	html += '<tr><td>Speed: ' 
	if (selected) {
	    if (Metric) {
	        html += Math.round(selected.speed * 1.852) + ' km/h';
	    } else {
	        html += selected.speed + ' kt';
	    }
	} else {
	    html += 'n/a';
	}
	html += '</td>';
	
	if (selected) {
        html += '<td>ICAO (hex): ' + selected.icao + '</td></tr>';
    } else {
        html += '<td>ICAO (hex): n/a</td></tr>'; // Something is wrong if we are here
    }
    
    html += '<tr><td>Track: ' 
	if (selected && selected.vTrack) {
	    html += selected.track + ' (' + normalizeTrack(selected.track, selected.vTrack)[1] +')';
	} else {
	    html += 'n/a';
	}

	html += '</td><td>Reg: '
	if (selected && selected.registration != '') {
	    html += '<a href="http://www.planespotters.net/Aviation_Photos/search.php?reg='+selected.registration +
	        '&o=14" target="_blank">' + selected.registration + '</a>';
	} else {
	    html += 'n/a';
	}
	html += '</td></tr>';

	html += '<tr><td colspan="' + columns + '" align="center">Lat/Long: ';
	if (selected && selected.vPosition) {
	    html += selected.latitude + ', ' + selected.longitude + '</td></tr>';
	    
	    // Let's show some extra data if we have site coordinates
	    if (SiteShow) {
            var siteLatLon  = new google.maps.LatLng(SiteLat, SiteLon);
            var planeLatLon = new google.maps.LatLng(selected.latitude, selected.longitude);
            var dist = google.maps.geometry.spherical.computeDistanceBetween(siteLatLon, planeLatLon);
            var bearing = google.maps.geometry.spherical.computeHeading(siteLatLon, planeLatLon);
            
            bearing = Math.round(bearing);
            if (bearing < 0) { bearing += 360; }
            
            if (Metric) {
                dist /= 1000;
            } else {
                dist /= 1852;
            }
            
            dist = (Math.round((dist)*10)/10).toFixed(1);
            html += '<tr><td colspan="' + columns + '">Distance from Site: ' + dist +
                (Metric ? ' km' : ' NM') + ' @ ' + bearing + '&deg;</td></tr>';
        } // End of SiteShow
	} else {
	    if (SiteShow) {
	        html += '<tr><td colspan="' + columns + '">Distance from Site: n/a ' + 
	            (Metric ? ' km' : ' NM') + '</td></tr>';
	    } else {
    	    html += 'n/a</td></tr>';
    	}
	}

	html += '</table>';
	
	document.getElementById('plane_detail').innerHTML = html;
}

// Right now we have no means to validate the speed is good
// Want to return (n/a) when we don't have it
// TODO: Edit C code to add a valid speed flag
// TODO: Edit js code to use said flag
function normalizeSpeed(speed, valid) {
	return speed	
}

// Returns back a long string, short string, and the track if we have a vaild track path
function normalizeTrack(track, valid){
	x = []
	if ((track > -1) && (track < 22.5)) {
		x = ["North", "N", track]
	}
	if ((track > 22.5) && (track < 67.5)) {
		x = ["North East", "NE", track]
	}
	if ((track > 67.5) && (track < 112.5)) {
		x = ["East", "E", track]
	}
	if ((track > 112.5) && (track < 157.5)) {
		x = ["South East", "SE", track]
	}
	if ((track > 157.5) && (track < 202.5)) {
		x = ["South", "S", track]
	}
	if ((track > 202.5) && (track < 247.5)) {
		x = ["South West", "SW", track]
	}
	if ((track > 247.5) && (track < 292.5)) {
		x = ["West", "W", track]
	}
	if ((track > 292.5) && (track < 337.5)) {
		x = ["North West", "NW", track]
	}
	if ((track > 337.5) && (track < 361)) {
		x = ["North", "N", track]
	}
	if (!valid) {
		x = [" ", "n/a", ""]
	}
	return x
}

function updateTableOfPlanes() {
    // Blank array
    data_tmp = [];

    // Planes on the table
    iPlanesTable = 0;
    // Planes that can be tracked to a lat/long
    iPlanesTrackable = 0;

    // Loop through all the planes
    for (var tablep in Planes) {
        // Pluck our plane
        var tableplane = Planes[tablep];

        // If the plane is not beyond 60 seconds, basically
        if (!tableplane.reapable) {
            // Count it on the table
            iPlanesTable++;

            // Data array for this plane to be passed to the table
            var tmp = {
                "altitude": tableplane.altitude,
                "speed": tableplane.speed,
                "track": tableplane.track,
                "latitude": tableplane.latitude,
                "longitude": tableplane.longitude,
                "flight": tableplane.flight,
                "icao": tableplane.icao,
                "messages": tableplane.messages,
                "seen": tableplane.seen,
                "country": tableplane.country,
                "country_short": tableplane.country_short,
                "type": tableplane.type,
                "operator": tableplane.operator,
                "registration": tableplane.registration,
                "squawk": tableplane.squawk,

                "flag": '<img src="' + remote_imgdir + 'Small_Flags/' + tableplane.country_flag + '" title="' + tableplane.country + '"  alt="' + tableplane.country + '" />',
                "operator_logo": '<img src="' + remote_imgdir + 'OperatorLogos/' + tableplane.operator + '.png" title="' + tableplane.operator + '"  alt="' + tableplane.operator + '" height="20" width="85" />',
                "silhouette": '<img src="' + remote_imgdir + 'SilhouettesLogos/' + tableplane.type + '.png" title="' + tableplane.type + '"  alt="' + tableplane.type + '" height="20" width="85" />',

                "DT_RowId":	tableplane.icao,
                "DT_RowClass":	tableplane.specialStyle,
            };

            // If all 0's then it is not something we want to show
            if (tableplane.squawk == '0000') {
                tmp["squawk"] = "";
            }

            // Push the data array to the tmp data table
            data_tmp.push(tmp);
        }

    };

    // Rebuild the table
    data_array = data_tmp;
    data_table.fnClearTable();
    data_table.fnAddData(data_array);

    // Update the document title to be something like "Dump1090 (1/2/3)"
    // 1 = Planes able to be tracked via Lat/Long
    // 2 = Planes that are shown on the table
    // 3 = All planes, including the ones preping to be reaped
    document.title = "Dump1090 (" + iPlanesTrackable + "/" + iPlanesTable + "/" + iPlanesTotal + ")";

    // If there is a squawk that is to be noted...
    if (SpecialSquawk) {
    	$('#SpecialSquawkWarning').css('display', 'inline');
    } else {
        $('#SpecialSquawkWarning').css('display', 'none');
    }
}

function onClickPlanes_table (hex) {
    if (hex && hex != '' && hex != "ICAO") {
		selectPlaneByHex(hex);
        updateTableOfPlanes();
		refreshSelected();
	}
}

// Credit goes to a co-worker that needed a similar functions for something else
// we get a copy of it free ;)
function setASC_DESC(iCol) {
	if(iSortCol==iCol) {
		bSortASC=!bSortASC;
	} else {
		bSortASC=bDefaultSortASC;
	}
}

function sortTable(szTableID,iCol) { 
	//if iCol was not provided, and iSortCol is not set, assign default value
	if (typeof iCol==='undefined'){
		if(iSortCol!=-1){
			var iCol=iSortCol;
		} else {
			var iCol=iDefaultSortCol;
		}
	}

	//retrieve passed table element
	var oTbl=document.getElementById(szTableID).tBodies[0];
	var aStore=[];

	//If supplied col # is greater than the actual number of cols, set sel col = to last col
	if (typeof oTbl.rows[0] !== 'undefined' && oTbl.rows[0].cells.length <= iCol) {
		iCol=(oTbl.rows[0].cells.length-1);
    }

	//store the col #
	iSortCol=iCol;

	//determine if we are delaing with numerical, or alphanumeric content
	var bNumeric = false;
	if (iSortCol == 3) { // Special sorting for Altitude-column
	    bNumeric = true;
	} else if ((typeof oTbl.rows[0] !== 'undefined') &&
	    (!isNaN(parseFloat(oTbl.rows[0].cells[iSortCol].textContent ||
	    oTbl.rows[0].cells[iSortCol].innerText)))) {
	    bNumeric = true;
	}

	//loop through the rows, storing each one inro aStore
	for (var i=0,iLen=oTbl.rows.length;i<iLen;i++){
		var oRow=oTbl.rows[i];
		vColData=bNumeric?parseFloat(oRow.cells[iSortCol].textContent||oRow.cells[iSortCol].innerText):String(oRow.cells[iSortCol].textContent||oRow.cells[iSortCol].innerText);
		
		// Sort empty altitude as 0
		if (iSortCol == 3 && (!vColData || vColData.length < 2)) {vColData = 0;}
		aStore.push([vColData,oRow]);
	}

	//sort aStore ASC/DESC based on value of bSortASC
	if (bNumeric) { //numerical sort
		aStore.sort(function(x,y){return bSortASC?x[0]-y[0]:y[0]-x[0];});
	} else { //alpha sort
		aStore.sort();
		if(!bSortASC) {
			aStore.reverse();
	    }
	}

	//rewrite the table rows to the passed table element
	for(var i=0,iLen=aStore.length;i<iLen;i++){
		oTbl.appendChild(aStore[i][1]);
	}
	aStore=null;
}

function selectPlaneByHex(hex) {
	// If SelectedPlane has something in it, clear out the selected
	if (SelectedPlane != null) {
		Planes[SelectedPlane].is_selected = false;
		Planes[SelectedPlane].funcClearLine();
		Planes[SelectedPlane].markerColor = MarkerColor;
		// If the selected has a marker, make it not stand out
		if (Planes[SelectedPlane].marker) {
			Planes[SelectedPlane].marker.setIcon(Planes[SelectedPlane].funcGetIcon());
		}
	}

	// If we are clicking the same plane, we are deselected it.
	if (String(SelectedPlane) != String(hex)) {
		// Assign the new selected
		SelectedPlane = hex;
		Planes[SelectedPlane].is_selected = true;
		// If the selected has a marker, make it stand out
		if (Planes[SelectedPlane].marker) {
			Planes[SelectedPlane].funcUpdateLines();
			Planes[SelectedPlane].marker.setIcon(Planes[SelectedPlane].funcGetIcon());
		}
	} else { 
		SelectedPlane = null;
	}
    refreshSelected();
    updateTableOfPlanes()
}

function resetMap() {
    // Reset localStorage values
    localStorage['CenterLat'] = CONST_CENTERLAT;
    localStorage['CenterLon'] = CONST_CENTERLON;
    localStorage['ZoomLvl']   = CONST_ZOOMLVL;
    
    // Try to read values from localStorage else use CONST_s
    CenterLat = Number(localStorage['CenterLat']) || CONST_CENTERLAT;
    CenterLon = Number(localStorage['CenterLon']) || CONST_CENTERLON;
    ZoomLvl   = Number(localStorage['ZoomLvl']) || CONST_ZOOMLVL;
    
    // Set and refresh
	GoogleMap.setZoom(parseInt(ZoomLvl));
	GoogleMap.setCenter(new google.maps.LatLng(parseFloat(CenterLat), parseFloat(CenterLon)));
	
	if (SelectedPlane) {
	    selectPlaneByHex(SelectedPlane);
	}
	
	// Set reset METAR-flag
	MetarReset = true;

	refreshSelected();
    updateTableOfPlanes();
	getMetar();
}

function drawCircle(marker, distance) {
    if (typeof distance === 'undefined') {
        return false;
        
        if (!(!isNaN(parseFloat(distance)) && isFinite(distance)) || distance < 0) {
            return false;
        }
    }
    
    distance *= 1000.0;
    if (!Metric) {
        distance *= 1.852;
    }
    
    // Add circle overlay and bind to marker
    var circle = new google.maps.Circle({
      map: GoogleMap,
      radius: distance, // In meters
      fillOpacity: 0.0,
      strokeWeight: 1,
      clickable: false,
      strokeOpacity: 0.3
    });
    circle.bindTo('center', marker, 'position');
}

function drawAntennaData(marker) {
    if (!AntennaDataShow) {
        if (AntennaDataPath && typeof AntennaDataPath !== 'undefined') {
            AntennaDataPath.setMap(null);
            AntennaDataPath = null;
        }
        return false;
    }    
    
    if (!marker && (SiteLat && SiteLon)) {
        marker = new google.maps.LatLng(parseFloat(SiteLat), parseFloat(SiteLon));
    }
    
    path = new Array();
    for (var i=0;i<360;i++) {
        if (typeof AntennaData[i] !== 'undefined') {
            var metricDist = AntennaData[i] * 1.852;
            path[i] = google.maps.geometry.spherical.computeOffset(marker, metricDist, i);
        } else {
            path[i] = marker;
        }
    }
    
    if (AntennaDataPath && typeof AntennaDataPath !== 'undefined') {
        AntennaDataPath.setMap(null);
        AntennaDataPath = null;
    }
    
    AntennaDataPath = new google.maps.Polygon({
        paths: path,
        fillColor: '#7f7f7f',
        fillOpacity: AntennaDataOpacity,
        strokeColor: '#7f7f7f',
        strokeWeight: 1,
        strokeOpacity: AntennaDataOpacity,
        clickable: false,
        zIndex: -99998
    });
    AntennaDataPath.setMap(GoogleMap);
}

/** gets csv of requested airport ICAOs as parameter **/
function getMetar(pMetarICAO) {
    if (!pMetarICAO || typeof pMetarICAO === 'undefined') { // as parameter
        if (!MetarIcaoCode || typeof MetarIcaoCode === 'undefined') { // from config.js
            return; // No metar to show
        } else {
            pMetarICAO = MetarIcaoCode;
        }
    }
    
    pMetarICAO = pMetarICAO.replace(/\s/g, "");
    url = 'http://weather.aero/dataserver_current/httpparam?dataSource=metars&' +
            'requestType=retrieve&format=csv&hoursBeforeNow=1&fields=raw_text&' +
            'mostRecentForEachStation=postfilter&stationString=' + pMetarICAO;
     
    //url = 'http://aviationweather.gov/adds/dataserver_current/httpparam?dataSource=metars&' +
    //        'requestType=retrieve&format=csv&hoursBeforeNow=1&fields=raw_text&' +
    //        'mostRecentForEachStation=postfilter&stationString=KCLT';
       
    xReader(url, function(data) {
        if (data.status == Number(200)) {
            csv = data.content.split("\n");
            csv.splice(0,6);
            html = "";
            for(i=0;i<csv.length;i++) {
                csv[i] = csv[i].replace(/,/g,"");
                html += csv[i];
                if (i < csv.length - 1) {
                    html += "<br>";
                }
            }
            document.getElementById('METAR').innerHTML = html;
            if (MetarReset) {
                $("#METAR").position({ my: "left bottom", at: "left+5 bottom-30", of: "#map_canvas" });
                $("#METAR").draggable({ containment: "document" });
                $("#METAR").addClass('ui-state-highlight');
                $('#METAR').show();
                MetarReset = false;
            }
        }
    });
}

/* Store objects as string to localStorage */
Storage.prototype.setObject = function(key, value) {
    this.setItem(key, JSON.stringify(value));
}

/* Get string objets from localStorage */
Storage.prototype.getObject = function(key) {
    var value = this.getItem(key);
    return value && JSON.parse(value);
}

/* Get size of object */
Object.size = function(obj) {
    var size = 0, key;
    for (key in obj) {
        if (obj.hasOwnProperty(key)) size++;
    }
    return size;
};
