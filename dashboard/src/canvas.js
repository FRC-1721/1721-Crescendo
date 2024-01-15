const canvas = document.getElementById("cool");
const gl = canvas.getContext("webgl");
if (!gl) {
    alert("yro;ue browser not habbing webgl >:(");
}
gl.clearColor(0, 1, 0, 1);
gl.clear(gl.COLOR_BUFFER_BIT);
