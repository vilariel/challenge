drawPixi = function(oldx, oldy, newx, newy) {
  var graphics = new PIXI.Graphics();
  graphics.lineStyle(2, 0xffd900, 1);
  var x0 = oldx / 12 * 500;
  var y0 = (1 - oldy / 12) * 500;
  var x1 = newx / 12 * 500;
  var y1 = (1 - newy / 12) * 500
  graphics.moveTo(x0, y0);
  graphics.lineTo(x1, y1);
  console.log('x0: ' + x0 + ', y0: ' + y0 + ', x1: ' + x1 + ', y1: ' + y1);
  stage.addChild(graphics);
  renderer.render(stage);
}
