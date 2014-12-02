By downloading and placing the 'enes100' folder in their local 'sketchbook/libraries' folder, students can then use pre-made functions to interface with the RF system.  There are really just three class functions they need to know

1)  .sendMessage sends a custom message to the vision system that will appear in the systems 'received' text box.

    rf.sendMessage("Team One Connected.");

2)  .receiveMarker requests a marker with the given id from the vision system, if the requested marker is received within the next 300ms it stores the data in the 'marker' object, and returns true.  If the marker is not received within 300ms, the function returns false.  The marker data can then be accessed by calling marker.id, marker.x, marker.y, marker.theta, or marker.time.

    if(rf.receiveMarker(&marker, 5)){
       Serial.print("I received marker 5, with x-coordinate: ");
       Serial.println(marker.x);
    }

3)  .resetServer()  resets the state of the server RF channel.  This should at minimum be called in the student's setup() function, to ensure the server is in the correct state when they begin their program.

     rf.resetServer();


All of these functions are demonstrated in the rf_client_example sketch, which can be accessed by selecting examples/rf_client_example.