var fieldRelative = false;

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

function ntToggle(event) {
    $(event.parentElement).toggleClass("disabled");
}

function put(items, path, value) {
    var [x, ...xs] = items;
    if (items.length > 1) {
        if ($("#nt-table #" + path + "-" + x).length == 0) {
            var div = $(" <div /> ", {
                id: path + "-" + x,
                class: "nt-div disabled",
            });
            $("<div />", {
                id: path + "-" + x + "_title",
                class: "nt-title",
                text: x,
                onClick: "ntToggle(this)"
            }).appendTo(div);
            $("<div />", {
                id: path + "-" + x + "-",
                class: "nt-div-container"
            }).appendTo(div);
            $("#nt-table #" + path + "-").append(div);
        }
        put(xs, path + "-" + x, value);
    } else {
        var item = $(" <div /> ", {
            id: path + "-" + x,
            class: "nt-item",
        });
        $(" <div /> ", {
            class: "nt-key",
            text: "path" + "-" + x
        }).appendTo(item);
        $(" <div /> ", {
            class: "nt-value",
            text: value
        }).appendTo(item);
        $("#nt-table #" + path + "-").append(item);
    }
}

function onValueChanged(key, value, isNew) {
    // key thing here: we're using the various NetworkTable keys as
    // the id of the elements that we're appending, for simplicity. However,
    // the key names aren't always valid HTML identifiers, so we use
    // the NetworkTables.keyToId() function to convert them appropriately

    if (isNew) {
        var [_, ...path] = key.split("/");
        put(path, "_", value)
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
        if (key.includes("desired")) {
            wheel = key.split("/").at(-1);
            $(".swerve-desired ." + wheel).css(
                "transform",
                "rotate(" + value + "deg)"
            );
        } else {
            wheel = key.split("/").at(-1).replace(" desired", "");
            $(".swerve ." + wheel).css("transform", "rotate(" + value + "deg)");
        }
    }
}
