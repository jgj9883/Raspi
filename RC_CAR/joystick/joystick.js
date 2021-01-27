var Joystick	= function(opts)
{
    opts = opts || {};
    this._container = opts.container || document.body;
    this._strokeStyle = opts.strokeStyle || 'cyan';
    this._lineStyle = opts.lineStyle || 'red';

    this._container.style.position = "relative";
    this._width   = this._container.offsetWidth;
    this._height  = this._container.offsetHeight;
    this._offsetX = this._container.offsetLeft;
    this._offsetY = this._container.offsetTop;
    this._centerX = this._width / 2;
    this._centerY = this._height / 2;
    this._radius  = (this._width < this._height) ? this._width / 2 : this._height / 2;
    this._movementRadius = this._radius / 2;
    this._movementRange	= 10;
    this._stickEl = opts.stickElement || this._buildJoystickStick();
    this._baseEl  = opts.baseElement  || this._buildJoystickBase();
    this._lineEl  = opts.lineElement  || this._buildJoystickLine();
    this._mouseSupport	= opts.mouseSupport !== undefined ? opts.mouseSupport : false;

    this._container.appendChild(this._baseEl);
    this._baseEl.style.position	= "absolute";
    this._baseEl.style.display	= "";
    this._baseEl.style.left = (this._centerX-this._baseEl.width /2)+"px";
    this._baseEl.style.top  = (this._centerY-this._baseEl.height /2)+"px";
    
    this._container.appendChild(this._stickEl);
    this._stickEl.style.position = "absolute";
    this._stickEl.style.display	= "";
    this._stickEl.style.left = (this._centerX-this._stickEl.width /2)+"px";
    this._stickEl.style.top  = (this._centerY-this._stickEl.height /2)+"px";
    
    this._container.appendChild(this._lineEl);
    this._lineEl.style.position	= "absolute";
    this._lineEl.style.display	= "none";
    
    this._pressed	= false;
    this._touchIdx	= null;
    this._baseX	= this._centerX;
    this._baseY	= this._centerY;
    this._stickX	= this._centerX;
    this._stickY	= this._centerY;
    this._numberOfFrames = 5;
    this._count		= 0;
    
    var __bind	= function(fn, me){ return function(){ return fn.apply(me, arguments); }; };
    this._$onTouchStart	= __bind(this._onTouchStart , this);
    this._$onTouchEnd	= __bind(this._onTouchEnd   , this);
    this._$onTouchMove	= __bind(this._onTouchMove  , this);
    this._container.addEventListener('touchstart', this._$onTouchStart, false);
    this._container.addEventListener('touchend'  , this._$onTouchEnd,   false);
    this._container.addEventListener('touchmove' , this._$onTouchMove,  false);
    if( this._mouseSupport ){
       this._$onMouseDown = __bind(this._onMouseDown	, this);
       this._$onMouseUp	  = __bind(this._onMouseUp	, this);
       this._$onMouseMove = __bind(this._onMouseMove	, this);
       this._container.addEventListener('mousedown', this._$onMouseDown, false);
       this._container.addEventListener('mouseup'  , this._$onMouseUp,   false);
       this._container.addEventListener('mousemove', this._$onMouseMove, false);
    }
}

Joystick.prototype.destroy	= function()
{
    this._container.removeChild(this._baseEl);
    this._container.removeChild(this._stickEl);
    this._container.removeChild(this._lineEl);
    
    this._container.removeEventListener('touchstart', this._$onTouchStart, false);
    this._container.removeEventListener('touchend' , this._$onTouchEnd, false);
    this._container.removeEventListener('touchmove', this._$onTouchMove, false);
    if( this._mouseSupport ){
       this._container.removeEventListener('mouseup', this._$onMouseUp, false);
       this._container.removeEventListener('mousedown', this._$onMouseDown, false);
       this._container.removeEventListener('mousemove', this._$onMouseMove, false);
    }
}

/**
 * @returns {Boolean} true if touchscreen is currently available, false otherwise
*/
Joystick.touchScreenAvailable	= function()
{
    return 'createTouch' in document ? true : false;
}

/**
 * microevents.js - https://github.com/jeromeetienne/microevent.js
*/
;(function(destObj){
    destObj.addEventListener	= function(event, fct){
       if(this._events === undefined) 	this._events	= {};
       this._events[event] = this._events[event]	|| [];
       this._events[event].push(fct);
       return fct;
    };
    destObj.removeEventListener	= function(event, fct){
       if(this._events === undefined) 	this._events	= {};
       if( event in this._events === false  )	return;
       this._events[event].splice(this._events[event].indexOf(fct), 1);
    };
    destObj.dispatchEvent = function(event /* , args... */){
       if(this._events === undefined) 	this._events	= {};
       if( this._events[event] === undefined )	return;
       var tmpArray	= this._events[event].slice(); 
       for(var i = 0; i < tmpArray.length; i++){
          var result = tmpArray[i].apply(this, Array.prototype.slice.call(arguments, 1))
          if( result !== undefined )	return result;
       }
       return undefined
    };
})(Joystick.prototype);

Joystick.prototype.deltaX = function(){ return this._stickX - this._baseX; }
Joystick.prototype.deltaY = function(){ return this._stickY - this._baseY; }

Joystick.prototype.up	= function(){
    if( this._pressed === false ) return false;
    var deltaX	= this.deltaX();
    var deltaY	= this.deltaY();
    if( deltaY >= 0 ) return false;
    if( Math.abs(deltaX) > 2*Math.abs(deltaY) )	return false;
    return true;
}
Joystick.prototype.down	= function(){
    if( this._pressed === false ) return false;
    var deltaX	= this.deltaX();
    var deltaY	= this.deltaY();
    if( deltaY <= 0 ) return false;
    if( Math.abs(deltaX) > 2*Math.abs(deltaY) )	return false;
    return true;	
}
Joystick.prototype.right = function(){
    if( this._pressed === false ) return false;
    var deltaX	= this.deltaX();
    var deltaY	= this.deltaY();
    if( deltaX <= 0 ) return false;
    if( Math.abs(deltaY) > 2*Math.abs(deltaX) )	return false;
    return true;	
}
Joystick.prototype.left	= function(){
    if( this._pressed === false ) return false;
    var deltaX	= this.deltaX();
    var deltaY	= this.deltaY();
    if( deltaX >= 0 ) return false;
    if( Math.abs(deltaY) > 2*Math.abs(deltaX) )	return false;
    return true;	
}

Joystick.prototype._onUp = function()
{
    this._count = this._numberOfFrames;
    this._lineEl.style.display	= "none";
    for (var i = 0; i < this._numberOfFrames; i++) {
       setTimeout(function(){returnToCenter();}, i*100);
    }
    //setTimeout(function(){returnToCenter();}, 100);
}

Joystick.prototype._onDown = function(x, y)
{
    this._pressed	= true; 
    this._stickX	= x;
    this._stickY	= y;
    this.constrainCircle();
    this._stickEl.style.left = (this._stickX - this._stickEl.width /2)+"px";
    this._stickEl.style.top  = (this._stickY - this._stickEl.height/2)+"px";
    this._lineEl.style.display = "";
    this.drawLine();
}

Joystick.prototype._onMove = function(x, y)
{
    if( this._pressed === true ){
       this._stickX	= x;
       this._stickY	= y;
       this.constrainCircle();
       this._stickEl.style.left = (this._stickX - this._stickEl.width /2)+"px";
       this._stickEl.style.top  = (this._stickY - this._stickEl.height/2)+"px";
       this.drawLine();
       this.sendMoveCommand(true);
    }
}

Joystick.prototype.drawLine = function() {
    var ctx		= this._lineEl.getContext('2d');
    ctx.clearRect(0,0,this._width, this._height);
    ctx.beginPath();
    ctx.strokeStyle	= this._lineStyle; 
    ctx.lineWidth	= 4;
    //ctx.moveTo(this._width/2, this._height/2);
    ctx.moveTo(this._lineEl.width/2, this._lineEl.height/2);
    ctx.lineTo(this._stickX, this._stickY); 
    ctx.stroke();
}


Joystick.prototype.constrainCircle = function() {
    var diffX = this.deltaX();
    var diffY = this.deltaY();
    var radial = Math.sqrt((diffX*diffX) + (diffY*diffY));
    if ( radial > this._movementRadius ) {
       this._stickX = this._baseX + (diffX / radial) * this._movementRadius;
       this._stickY = this._baseY + (diffY / radial) * this._movementRadius;
    }
}

Joystick.prototype.sendMoveCommand = function(enableAck) {
    //First convert to cartesian coordinates
    var cartX = this.deltaX() / this._movementRadius * this._movementRange;
    var cartY = this.deltaY() / this._movementRadius * this._movementRange;
    var radial = parseInt(Math.sqrt((cartX*cartX) + (cartY*cartY)));
    var angle = - Math.atan2(cartY, cartX);
    motion("speed="+radial+"&angle="+angle, enableAck);
}
Joystick.prototype.getSpeed = function() {
    //First convert to cartesian coordinates
    var cartX = this.deltaX() / this._movementRadius * this._movementRange;
    var cartY = this.deltaY() / this._movementRadius * this._movementRange;
    return parseInt(Math.sqrt((cartX*cartX) + (cartY*cartY)));
}
Joystick.prototype.getAngle = function() {
    //First convert to cartesian coordinates
    var cartX = this.deltaX() / this._movementRadius * this._movementRange;
    var cartY = this.deltaY() / this._movementRadius * this._movementRange;
    return - Math.atan2(cartY, cartX);
}

Joystick.prototype._onMouseUp	= function(event)
{
    return this._onUp();
}

Joystick.prototype._onMouseDown	= function(event)
{
    var x = event.clientX - this._offsetX;
    var y = event.clientY - this._offsetY;
    return this._onDown(x, y);
}

Joystick.prototype._onMouseMove	= function(event)
{
    var x = event.clientX - this._offsetX;
    var y = event.clientY - this._offsetY;
    return this._onMove(x, y);
}

Joystick.prototype._onTouchStart = function(event)
{
    // if there is already a touch inprogress do nothing
    if( this._touchIdx !== null ) return;
    
    // notify event for validation
    var isValid	= this.dispatchEvent('touchStartValidation', event);
    if( isValid === false ) return;
    
    event.preventDefault();
    // get the first who changed
    var touch = event.changedTouches[0];
    // set the touchIdx of this joystick
    this._touchIdx = touch.identifier;
    
    // forward the action
    var x = touch.pageX - this._offsetX + document.body.scrollLeft;
    var y = touch.pageY - this._offsetY + document.body.scrollTop;
    return this._onDown(x, y);
}

Joystick.prototype._onTouchEnd	= function(event)
{
    // if there is no touch in progress, do nothing
    if( this._touchIdx === null )	return;
    
    // try to find our touch event
    var touchList = event.changedTouches;
    for(var i = 0; i < touchList.length && touchList[i].identifier !== this._touchIdx; i++);
    // if touch event isnt found, 
    if( i === touchList.length)	return;
    
    // reset touchIdx - mark it as no-touch-in-progress
    this._touchIdx = null;
    
    //??????
    // no preventDefault to get click event on ios
    event.preventDefault();
    
    return this._onUp();
}

Joystick.prototype._onTouchMove	= function(event)
{
    // if there is no touch in progress, do nothing
    if( this._touchIdx === null )	return;
    
    // try to find our touch event
    var touchList	= event.changedTouches;
    for(var i = 0; i < touchList.length && touchList[i].identifier !== this._touchIdx; i++ );
    // if touch event with the proper identifier isnt found, do nothing
    if( i === touchList.length)	return;
    var touch	= touchList[i];
    
    event.preventDefault();
    
    var x = touch.pageX - this._offsetX + document.body.scrollLeft;
    var y = touch.pageY - this._offsetY + document.body.scrollTop;
    return this._onMove(x, y);
}


/**
 * build the canvas for joystick base
 */
Joystick.prototype._buildJoystickBase	= function()
{
    var canvas	= document.createElement( 'canvas' );
    canvas.width	= this._radius;
    canvas.height	= this._radius;
    var ctx		= canvas.getContext('2d');
    ctx.beginPath(); 
    ctx.strokeStyle	= this._strokeStyle; 
    ctx.lineWidth	= 2; 
    ctx.arc( canvas.width/2, canvas.width/2, canvas.width/2 - ctx.lineWidth, 0, Math.PI*2, true); 
    ctx.stroke();
    return canvas;
}

/**
 * build the canvas for joystick stick
 */
Joystick.prototype._buildJoystickStick	= function()
{
    var canvas	= document.createElement( 'canvas' );
    canvas.width	= this._radius / 2;
    canvas.height	= this._radius / 2;
    var ctx		= canvas.getContext('2d');
    ctx.beginPath(); 
    ctx.fillStyle	= this._strokeStyle; 
    ctx.lineWidth	= 2; 
    ctx.arc( canvas.width/2, canvas.width/2, canvas.width/2 - ctx.lineWidth, 0, Math.PI*2, true); 
    ctx.fill();
    return canvas;
}

/**
 * build the canvas for line from center to joystick
 */
Joystick.prototype._buildJoystickLine	= function()
{
    var canvas	= document.createElement( 'canvas' );
    canvas.width	= this._width;
    canvas.height	= this._height;
    return canvas;
}

function returnToCenter() {
    joystick._stickX -= (joystick._stickX - joystick._baseX) / joystick._count;
    joystick._stickY -= (joystick._stickY - joystick._baseY) / joystick._count;
    joystick._count--;
    
    if (joystick._count == 0) {
       joystick._pressed	= false; 
       joystick._stickX	= joystick._baseX;
       joystick._stickY	= joystick._baseY;
    }
    joystick._stickEl.style.left = (joystick._stickX - joystick._stickEl.width /2)+"px";
    joystick._stickEl.style.top	 = (joystick._stickY - joystick._stickEl.height/2)+"px";
    joystick.sendMoveCommand(false);
    //if (joystick._count > 0)
    //   setTimeout(function(){returnToCenter();}, 100);
}
