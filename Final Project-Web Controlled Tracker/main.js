
function control(str, file){
  var hr = new XMLHttpRequest();
  // Create some variables we need to send to our PHP file
  var url = file;
  var vars = 'command='+str;
  hr.open("POST", url, true);
  // Set content type header information for sending url encoded variables in the request
  hr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
  // Access the onreadystatechange event for the XMLHttpRequest object
  hr.onreadystatechange = function() {
    if(hr.readyState == 4 && hr.status == 200) {
      var return_data = hr.responseText.split("|");
    document.getElementById("status").value = return_data[0];
    }
  }
  // Send the data to PHP now... and wait for response to update the status div
  hr.send(vars); // Actually execute the request
  document.getElementById("status").value = "processing...";
}
function manual(){
  alert("Manual Control Now Enabled, Controls Have Been Unlocked!");
	document.getElementById("manual").disabled = true;
	document.getElementById("auto").disabled = false;
  document.getElementById("up").disabled = false;
  document.getElementById("down").disabled = false;
  document.getElementById("left").disabled = false;
  document.getElementById("right").disabled = false;
  var hr = new XMLHttpRequest();
  var url = 'main.php';
  var vars = 'command=manual';
  hr.open("POST", url, true);
  hr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
	hr.onreadystatechange = function() {
      if(hr.readyState == 4 && hr.status == 200) {
          loop = setInterval(function(){
          jQuery.get('data.txt', function(data){
            var lines = data.split("\n");
            $('#tempval').text(lines[0]);
            $('#lightval').text(lines[1]);
            $('#xval').text(lines[2]);
            $('#yval').text(lines[3]);
          });
          if (document.getElementById("manual").disabled == false)
            clearInterval(loop);
        }, 450);


      }
    }
  hr.send(vars);
}
function auto(){
  alert("Automatic Control is now Enabled!");
	document.getElementById("auto").disabled = true;
	document.getElementById("manual").disabled = false;
  document.getElementById("up").disabled = true;
  document.getElementById("down").disabled = true;
  document.getElementById("left").disabled = true;
  document.getElementById("right").disabled = true;
    var hr = new XMLHttpRequest();
    var url = 'main.php';
    var vars = 'command=auto';
		var loop = setInterval(function(){
					clearInterval(loop);});
    hr.open("POST", url, true);
    hr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
    hr.onreadystatechange = function() {
      if(hr.readyState == 4 && hr.status == 200) {
          loop = setInterval(function(){
          jQuery.get('data.txt', function(data){
            var lines = data.split("\n");
            $('#tempval').text(lines[0]);
            $('#lightval').text(lines[1]);
            $('#xval').text(lines[2]);
            $('#yval').text(lines[3]);
          });
          if (document.getElementById("auto").disabled == false)
            clearInterval(loop);
        }, 550);


      }
    }
    hr.send(vars);
		
}

