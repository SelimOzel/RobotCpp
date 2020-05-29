window.onload = function()
{
    // Modify background color
    //document.body.classList.add('bg');
    div11 = document.getElementById("grid-1-1");
    div12 = document.getElementById("grid-1-2");
    div21 = document.getElementById("grid-2-1");
    div22 = document.getElementById("grid-2-2");
    data = {x:[1,2,3,4,5],y:[1,2,4,8,16]};
    trace = [data];
    Plotly.plot(div11, trace);
    Plotly.plot(div12, trace);
    Plotly.plot(div21, trace);
    Plotly.plot(div22, trace);
};