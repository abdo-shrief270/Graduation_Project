let control=document.getElementById('control');
let moving=document.getElementById('moving');
let dis=document.getElementById('dis');
let appear=document.getElementById('appear');
moving.classList.add('hide');
control.onclick=function()
{   
    moving.classList.toggle('hide');
    dis.classList.toggle('hide');
    appear.classList.toggle('hide');
    control.classList.toggle('anime');  
}
