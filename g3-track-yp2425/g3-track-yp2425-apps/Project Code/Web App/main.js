let live=document.getElementById('live');
let more=document.getElementById('more');
let square=document.getElementById('square');
let mm=document.getElementById('mm');
let nn=document.getElementById('nn');
let stream=document.getElementById('stream');
let squaree=document.getElementById('squaree');
let circle=document.getElementById('circle')

if (window.innerWidth >= 500) {

square.onclick = function () {
    square.classList.toggle('hide');
    squaree.classList.remove('hide');
    mm.style.fontSize='10px';
    mm.style.transition='1s';
    live.style.width='550px';
    live.style.transition='1s';
    more.style.width='90px';
    more.style.transition='1s'; 
    nn.style.fontSize='32px';
    nn.style.transition='1s'; 
    }

squaree.onclick=function(){
    squaree.classList.toggle('hide');
    square.classList.remove('hide');
    mm.style.fontSize='32px';
    mm.style.transition='1s';
    live.style.width='90px';
    live.style.transition='1s';
    more.style.width='550px';
    more.style.transition='1s'; 
    nn.style.fontSize='10px';
    nn.style.transition='1s';         
    }
}

else{
    square.onclick=function(){
        square.classList.toggle('hide');
        squaree.classList.remove('hide');
        mm.style.fontSize='6px';
        mm.style.transition='1s';
        nn.style.fontWeight='bold';
        nn.style.fontSize='16px';
        nn.style.transition='1s';
        more.style.width='60px';
        more.style.transition='1s'; 
        live.style.width='300px';
        live.style.transition='1s';
        circle.style.width='12px';
        circle.style.height='12px';
        circle.style.transition='1s';
    }
    squaree.onclick=function(){
        squaree.classList.toggle('hide');
        square.classList.remove('hide');
        mm.style.fontSize='16px';
        mm.style.transition='1s';
        nn.style.fontSize='6px';
        nn.style.fontWeight='bold';
        nn.style.transition='1s';
        more.style.width='300px';
        more.style.transition='1s';
        live.style.width='60px';
        live.style.transition='1s';
        circle.style.width='6px';
        circle.style.height='6px';
        circle.style.transition='1s';
    }
}
