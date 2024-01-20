$(function () {
    // sets a function that will be called when the websocket connects/disconnects
    NetworkTables.addWsConnectionListener(onNetworkTablesConnection, true);

    // sets a function that will be called when the robot connects/disconnects
    NetworkTables.addRobotConnectionListener(onRobotConnection, true);

    // sets a function that will be called when any NetworkTables key/value changes
    NetworkTables.addGlobalListener(onValueChanged, true);

    /* // Doesn't work, using img, src=<ip>
    loadCameraOnConnect({
        container: "#cam1", // id of camera div
        proto: null, // url scheme
        host: "10.17.21.11", // ip
        port: 5800,
        image_url: "/",
        data_url: "/",
        wait_img: require("./assets/no_signal.png"),
        error_img: require("./assets/error.png"),
        attrs: {
            width: 320,
            height: 240,
        },
    });
    */
});

function onRobotConnection(connected) {
    $("#robotstate").text(connected ? "Connected" : "Disconnected");
    $("#robotAddress").text(
        connected ? NetworkTables.getRobotAddress() : "Disconnected"
    );
}

function onNetworkTablesConnection(connected) {
    if (connected) {
        $("#connectstate").text("Connected");

        // clear the table
        $("#nt tbody > tr").remove();
    } else {
        $("#connectstate").text("Disconnected");
    }
}

function onValueChanged(key, value, isNew) {
    // key thing here: we're using the various NetworkTable keys as
    // the id of the elements that we're appending, for simplicity. However,
    // the key names aren't always valid HTML identifiers, so we use
    // the NetworkTables.keyToId() function to convert them appropriately

    if (isNew) {
        var tr = $('<div class="table-info"></div>').appendTo($("#nt-table"));
        $('<div class="table-label"></div>').text(key).appendTo(tr);
        $('<div class="table-area"></div>')
            .attr("id", NetworkTables.keyToId(key))
            .text(value)
            .appendTo(tr);
    } else {
        // similarly, use keySelector to convert the key to a valid jQuery
        // selector. This should work for class names also, not just for ids
        $("#" + NetworkTables.keySelector(key)).text(value);
    }

    if (key.includes("/SmartDashboard/Audio")) {
        countDownAlerts(key, value);
    }

    if (key === "/SmartDashboard/Autonomous/options") {
        var options = NetworkTables.getValue(
            "/SmartDashboard/Autonomous/options"
        );
    }

    if (key.includes("/SmartDashboard/Swerve/")) {
        wheel = key.split("/").at(-1);
        console.log(wheel);
        $("#" + wheel).css("transform", "rotate(" + value + "deg)");
    }
}
