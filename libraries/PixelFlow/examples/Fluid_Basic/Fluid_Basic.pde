/**
 * 
 * PixelFlow | Copyright (C) 2016 Thomas Diewald - http://thomasdiewald.com
 * 
 * A Processing/Java library for high performance GPU-Computing (GLSL).
 * MIT License: https://opensource.org/licenses/MIT
 * 
 */



import com.thomasdiewald.pixelflow.java.DwPixelFlow;
import com.thomasdiewald.pixelflow.java.fluid.DwFluid2D;

import controlP5.Accordion;
import controlP5.ControlP5;
import controlP5.Group;
import controlP5.RadioButton;
import controlP5.Toggle;
import processing.core.*;
import processing.opengl.PGraphics2D;
import controlP5.CallbackListener;
import controlP5.CallbackEvent;
import controlP5.*;

import processing.serial.*;

Serial port;
Serial port2;

// TODO:
// 1. Making uniform velocity profile instead of round - CHECK
// 2. No residue of earlier pipe - CHECK
// 3. Getting actual velocity values - CHECK
// 4. 3-dimensional flow - can do computationally
// 5. See the parabolic profile more clearly - stop it from being turbulent - yes
// 6. Need to create a pressure gradient (?)
// 7. Got correct values for parabolic profile - CHECK

  // This example shows a very basic fluid simulation setup. 
  // Multiple emitters add velocity/temperature/density each iteration.
  // Obstacles are added at startup, by just drawing into a usual PGraphics object.
  // The same way, obstacles can be added/removed dynamically.
  //
  // additionally some locations add temperature to the scene, to show how
  // buoyancy works.
  //
  //
  // controls:
  //
  // LMB: add Density + Velocity
  // MMB: draw obstacles
  // RMB: clear obstacles

  float[] velocities;
  
  public class MyFluidData implements DwFluid2D.FluidData{
    
     public float px, py, vy, radius, vscale, r, g, b, intensity, temperature;
     public float vx = 100f;
     public float dpdx = (P2 - P1)/pipeLength;
     public float viscosity=100f;
     public float density = 100f;
    
    // update() is called during the fluid-simulation update step.
    @Override
    public void update(DwFluid2D fluid) {
     
      // add impulse: density + velocity
      intensity = 1.0f;
      px = viewport_w/3;
      py = viewport_h * 0.6;
      radius = pipeRadius;
      r = 0.0f;
      g = 0.3f;
      b = 1.0f;
      vy = 0f;
      
      PGraphics2D pg_entrance = (PGraphics2D) createGraphics(viewport_w, viewport_h, P2D);
      pg_entrance.smooth(0);
      pg_entrance.beginDraw();
      pg_entrance.clear();
      pg_entrance.rect(px, py-pipeRadius, pipeLength/2, 2*pipeRadius); //density of fluid in the pipe
      pg_entrance.endDraw();
      
      fluid.addDensity(pg_entrance, 1, 1, 1);
      
      //fluid.addDensity(px, py, radius, r, g, b, intensity);
      //fluid.addVelocity(px, py, radius, vx, vy);
      for (int i = (int) px; i < (int) px + 15; i++) {
        for (int j = (int) py - (int) pipeRadius; j < py + pipeRadius; j++){
          int y = abs(j - (int) py);
         // println(y);
           dpdx = (P2 - P1)/pipeLength;
           dpdx = -1;
          float v = vx / (pipeRadius*pipeRadius) * (y*y - pipeRadius*pipeRadius) * dpdx;
        //  println(v);
          //println(vx);
          fluid.addVelocity(i, j, 2, v, 0);
        }
      }

      /*if((fluid.simulation_step) % 200 == 0){
        temperature = 50f;
        fluid.addTemperature(px, py, radius, temperature);
      }*/
      
      temperature = 20f;
      fluid.addTemperature(px, py, radius, temperature);
     
      /*// add impulse: density + temperature
      float animator = sin(fluid.simulation_step*0.01f);
 
      intensity = 1.0f;
      px = 2*width/3f;
      py = 150;
      radius = 50;
      r = 1.0f;
      g = 0.0f;
      b = 0.3f;
      fluid.addDensity(px, py, radius, r, g, b, intensity);
      
      temperature = animator * 20f;
     // temperature = 20f;
     // fluid.addTemperature(px, py, radius, temperature);
      
      
      // add impulse: density 
      px = 1*width/3f;
      py = height-2*height/3f;
      radius = 50.5f;
      r = g = b = 64/255f;
      intensity = 1.0f;
      fluid.addDensity(px, py, radius, r, g, b, intensity, 3);

      
      boolean mouse_input = !cp5.isMouseOver() && mousePressed && !obstacle_painter.isDrawing();
      
      // add impulse: density + velocity
      if(mouse_input && mouseButton == LEFT){
        radius = 15;
        vscale = 15;
        px     = mouseX;
        py     = height-mouseY;
        vx     = (mouseX - pmouseX) * +vscale;
        vy     = (mouseY - pmouseY) * -vscale;
        
        fluid.addDensity(px, py, radius, 0.25f, 0.0f, 0.1f, 1.0f);
        fluid.addVelocity(px, py, radius, vx, vy);
      }*/
     
    }
  }
  
  
  int viewport_w = 800;
  int viewport_h = 800;
  int fluidgrid_scale = 1;
  
  int gui_w = 200;
  int gui_x = 20;
  int gui_y = 20;
  
  float pipeRadius = 100;
  float pipeLength = 200;
  float entranceVelocity = 100;
  float platesOrPipe = 0;
  float xpos = 400;
  float ypos = 400;            
  int xCenter = 900;
  int yCenter = 300;
  float yTop = yCenter + pipeRadius;
  float yBottom = yCenter - pipeRadius;
  
  float P1 = 0.0;
  float P2 = 0.0;
  
  boolean SERIAL = false;
  
  ControlP5 cp5;
  public RadioButton r;
  
       
  DwFluid2D fluid;
  ObstaclePainter obstacle_painter;
  MyFluidData cb_fluid_data;
 
  // render targets
  PGraphics2D pg_fluid;
  //texture-buffer, for adding obstacles
  PGraphics2D pg_obstacles;
  //For identifying your location
  PGraphics2D pg_location;
  //For showing the obstacles pictorially
  PGraphics2D pg_obstacle_drawing;
  
  // some state variables for the GUI/display
  int     BACKGROUND_COLOR           = 0;
  boolean UPDATE_FLUID               = true;
  boolean DISPLAY_FLUID_TEXTURES     = true;
  boolean DISPLAY_FLUID_VECTORS      = true;
  int     DISPLAY_fluid_texture_mode = 3;
  

  public void settings() {
    size(viewport_w, viewport_h, P2D);
    smooth(2);
  }
  
  public void setup() {
   
    // main library context
    DwPixelFlow context = new DwPixelFlow(this);
    context.print();
    context.printGL();

    // fluid simulation
    fluid = new DwFluid2D(context, viewport_w, viewport_h, fluidgrid_scale);
    
    // set some simulation parameters
    fluid.param.dissipation_density     = 0.999f;
    fluid.param.dissipation_velocity    = 0.99f;
    fluid.param.dissipation_temperature = 0.80f;
    fluid.param.vorticity               = 0.00f;
    //Temperature should not cause fluid to float up
    fluid.param.apply_buoyancy = false;
    
    // interface for adding data to the fluid simulation
    cb_fluid_data = new MyFluidData();
    fluid.addCallback_FluiData(cb_fluid_data);
   
    // pgraphics for fluid
    pg_fluid = (PGraphics2D) createGraphics(viewport_w, viewport_h, P2D);
    pg_fluid.smooth(4);
    pg_fluid.beginDraw();
    pg_fluid.background(BACKGROUND_COLOR);
    pg_fluid.endDraw();
    
    // pgraphics for obstacles
    pg_obstacles = (PGraphics2D) createGraphics(viewport_w, viewport_h, P2D);
    pg_obstacles.smooth(0);
    pg_obstacles.beginDraw();
    pg_obstacles.clear();
    
    //pgraphics for location
    pg_location = (PGraphics2D) createGraphics(viewport_w, viewport_h, P2D);
    pg_location.smooth(0);
    pg_location.beginDraw();
    pg_location.clear();
    
        // pgraphics for obstacles
    pg_obstacle_drawing = (PGraphics2D) createGraphics(viewport_w, viewport_h, P2D);
    pg_obstacle_drawing.smooth(0);
    pg_obstacle_drawing.beginDraw();
    pg_obstacle_drawing.clear();
    
    
    
    /*
    // circle-obstacles
    pg_obstacles.strokeWeight(10);
    pg_obstacles.noFill();
    pg_obstacles.noStroke();
    pg_obstacles.fill(64);
    float radius;
    radius = 100;
    pg_obstacles.ellipse(1*width/3f,  2*height/3f, radius, radius);
    radius = 150;
    pg_obstacles.ellipse(2*width/3f,  2*height/4f, radius, radius);
    radius = 200;
    pg_obstacles.stroke(64);
    pg_obstacles.strokeWeight(10);
    pg_obstacles.noFill();
    pg_obstacles.ellipse(1*width/2f,  1*height/4f, radius, radius);
    // border-obstacle
    pg_obstacles.strokeWeight(20);
    pg_obstacles.stroke(64);
    pg_obstacles.noFill();
    pg_obstacles.rect(0, 0, pg_obstacles.width, pg_obstacles.height;
    */
    pg_obstacles.endDraw(); 
    
    // class, that manages interactive drawing (adding/removing) of obstacles
    obstacle_painter = new ObstaclePainter(pg_obstacles);
    
    createGUI();
    
    frameRate(60);
    
    velocities = fluid.getVelocity(null, (int) xpos, (int) ypos, 1, 1);
    for (int i = 0; i < velocities.length; i++){
      println(velocities[i]);
    }
    
    //SERIAL LIST
    
    if(SERIAL) {
     println(Serial.list());

     // Open the port that the Arduino board is connected to (in this case #0)
     // Make sure to open the port at the same speed Arduino is using (9600bps)
     port = new Serial(this, Serial.list()[0], 9600);
     port2 = new Serial(this, Serial.list()[1]. 9600); 
   
    }
    
  }
  


  public void draw() {    
    
    // update simulation
    if(UPDATE_FLUID){
      fluid.addObstacles(pg_obstacles);
      fluid.update(); //This seems silly
      
    }

    //Print out integer values of fluid (this slows it down apparently)
    
    if (!SERIAL) {
       velocities = fluid.getVelocity(velocities, (int) xpos, ((int) viewport_h - (int) ypos), 1, 1);
      println("X pos: " + xpos + " X velocity: " + velocities[0]);
      println("Y pos: " + ypos + " Y velocity: " + velocities[1]);
    } else {
      //PRINT VELOCITIES TO SERIAL
      
      int v = (int) (velocities[0]*velocities[0] + velocities[1]* velocities[1]);
      port.write(v); //print velocity for now (paddle - print pressure)
      port2.write(v);
    }
    //8 force levels - write 1 byte (x velocity, y velocity) 4 4 
    //function to map x velocity and y velocities to forces (log?)
    //function to map pressure 8 (log scale)
 
    
    // clear render target
    pg_fluid.beginDraw();
    pg_fluid.background(BACKGROUND_COLOR);
    pg_fluid.endDraw();
    
    
    
    
    // render fluid stuff
    if(DISPLAY_FLUID_TEXTURES){
      // render: density (0), temperature (1), pressure (2), velocity (3)
      fluid.renderFluidTextures(pg_fluid, DISPLAY_fluid_texture_mode);
    }
    
    if(DISPLAY_FLUID_VECTORS){
      // render: velocity vector field
      fluid.renderFluidVectors(pg_fluid, 10);
    }
    
    pg_fluid.beginDraw();
    pg_fluid.endDraw();
    
    pg_obstacle_drawing.beginDraw();
    pg_obstacle_drawing.clear();
   int xStart = viewport_w/3;
  int yStart = (int) (viewport_h * 0.4); 
  int yTop = yStart + (int) pipeRadius;
  int yBottom = yStart - (int) pipeRadius;
  platesOrPipe = (r.getArrayValue()[1] > 0) ? 1 : 0;
  if(platesOrPipe==1){
    pg_obstacle_drawing.noFill();
    pg_obstacle_drawing.stroke(255);
    pg_obstacle_drawing.ellipse(xStart+pipeLength, yStart, pipeRadius/2, 2*pipeRadius);
    pg_obstacle_drawing.stroke(255);
    pg_obstacle_drawing.line(xStart, yTop, xStart + pipeLength, yTop);
    pg_obstacle_drawing.stroke(255);
    pg_obstacle_drawing.line(xStart, yBottom, xStart + pipeLength, yBottom);
    pg_obstacle_drawing.noFill();
    pg_obstacle_drawing.arc(xStart, yStart, pipeRadius/2, 2*pipeRadius, HALF_PI, PI+HALF_PI);
  } else {
    pg_obstacle_drawing.stroke(255);
    pg_obstacle_drawing.line(xStart, yTop, xStart + pipeLength, yTop);
    pg_obstacle_drawing.stroke(255);
    pg_obstacle_drawing.line(xStart, yBottom, xStart + pipeLength, yBottom);
    
  }

    
  
    pg_obstacle_drawing.endDraw();
    
    pg_obstacles.beginDraw();
    pg_obstacles.clear();
    
    pg_obstacles.fill(255);
    pg_obstacles.rect(xStart, 0, pipeLength, yTop - 2* pipeRadius); //Top barrier
    pg_obstacles.rect(xStart, yBottom + 2*pipeRadius, pipeLength, yTop); //Bottom barrier
    pg_obstacles.rect(0, 0, xStart, viewport_h);
    pg_obstacles.endDraw();
     
   pg_location.beginDraw();   
   pg_location.clear();
   pg_location.fill(140);

   pg_location.ellipse(xpos, ypos, 10, 10);
   pg_location.endDraw();

    // display  

        image(pg_obstacles, 0, 0);
    image(pg_fluid    , 0, 0);
    image(pg_location, 0, 0);
        image(pg_obstacle_drawing, 0, 0);

    
    
          //Drawing the pipe
      
 
    //obstacle_painter.displayBrush(this.g);
    
    // info
    String txt_fps = String.format(getClass().getName()+ "   [size %d/%d]   [frame %d]   [fps %6.2f]", fluid.fluid_w, fluid.fluid_h, fluid.simulation_step, frameRate);
    surface.setTitle(txt_fps);
   
  
  }
  


  public void mousePressed(){
    //if(mouseButton == CENTER ) obstacle_painter.beginDraw(1); // add obstacles
    //if(mouseButton == RIGHT  ) obstacle_painter.beginDraw(2); // remove obstacles
    xpos = mouseX;
    ypos = mouseY;
  }
  
  public void mouseDragged(){
   // obstacle_painter.draw();
   xpos = mouseX;
   ypos = mouseY;
  }
  
  public void mouseReleased(){
    //obstacle_painter.endDraw();
    xpos = mouseX;
    ypos = mouseY;
  }
  

  public void fluid_resizeUp(){
    fluid.resize(width, height, fluidgrid_scale = max(1, --fluidgrid_scale));
  }
  public void fluid_resizeDown(){
    fluid.resize(width, height, ++fluidgrid_scale);
  }
  public void fluid_reset(){
    fluid.reset();
  }
  public void fluid_togglePause(){
    UPDATE_FLUID = !UPDATE_FLUID;
  }
  public void fluid_displayMode(int val){
    DISPLAY_fluid_texture_mode = val;
    DISPLAY_FLUID_TEXTURES = DISPLAY_fluid_texture_mode != -1;
  }
  public void fluid_displayVelocityVectors(int val){
    DISPLAY_FLUID_VECTORS = val != -1;
  }

  public void keyReleased(){
    if(key == 'p') fluid_togglePause(); // pause / unpause simulation
    if(key == '+') fluid_resizeUp();    // increase fluid-grid resolution
    if(key == '-') fluid_resizeDown();  // decrease fluid-grid resolution
    if(key == 'r') fluid_reset();       // restart simulation
    
    //if(key == '1') DISPLAY_fluid_texture_mode = 0; // density
    if(key == '3') DISPLAY_fluid_texture_mode = 2; // pressure
    if(key == '4') DISPLAY_fluid_texture_mode = 3; // velocity
    
    if(key == 'q') DISPLAY_FLUID_TEXTURES = !DISPLAY_FLUID_TEXTURES;
    if(key == 'w') DISPLAY_FLUID_VECTORS  = !DISPLAY_FLUID_VECTORS;
    
    if (keyCode == 40){
      //Down arrow
        ypos+=5;
    } else if (keyCode == 38) {
      //Up arrow 
      ypos-=5;
    } else if (keyCode == 37){
      xpos-=5; //left arrow
    } else if (keyCode == 39) {
      xpos+=5; //right arrow
    }
  }
 

  
  public void createGUI(){
    cp5 = new ControlP5(this);
    
    int sx, sy, px, py, oy;
    
    sx = 100; sy = 14; oy = (int)(sy*1.5f);
    

    ////////////////////////////////////////////////////////////////////////////
    // GUI - FLUID
    ////////////////////////////////////////////////////////////////////////////
    Group group_fluid = cp5.addGroup("fluid");
    {
      group_fluid.setHeight(20).setSize(gui_w, 300)
      .setBackgroundColor(color(16, 180)).setColorBackground(color(16, 180));
      group_fluid.getCaptionLabel().align(CENTER, CENTER);
      
      px = 10; py = 15;
      
      cp5.addButton("reset").setGroup(group_fluid).plugTo(this, "fluid_reset"     ).setSize(80, 18).setPosition(px    , py);
      cp5.addButton("+"    ).setGroup(group_fluid).plugTo(this, "fluid_resizeUp"  ).setSize(39, 18).setPosition(px+=82, py);
      cp5.addButton("-"    ).setGroup(group_fluid).plugTo(this, "fluid_resizeDown").setSize(39, 18).setPosition(px+=41, py);
          
      ControlListener c = new ControlListener(){
        public void controlEvent(ControlEvent theEvent){
          println("Event value!");
          println(theEvent.getValue());
          platesOrPipe = theEvent.getValue();
          fluid_reset();
        }
      };    
      
       CallbackListener cb = new CallbackListener() {
        public void controlEvent(CallbackEvent theEvent) {
          fluid_reset(); //No longer works at all
        }
      };
      
      r = cp5.addRadioButton("radioButton")
         .setGroup(group_fluid)
         .setPosition(px - (82+41), py+=30)
         .addItem("Plate",0)
         .addItem("Pipe",1)
         .addListener(c);
      
      
      
      px = 10;
      cp5.addSlider("velocity").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=(int)(oy*1.5f))
          .setRange(0, 1).setValue(fluid.param.dissipation_velocity).plugTo(fluid.param, "dissipation_velocity"); //actually viscosity
      
      cp5.addSlider("density").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 1).setValue(fluid.param.dissipation_density).plugTo(fluid.param, "dissipation_density");
          
      cp5.addSlider("pipeRadius").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(50, 300).setValue(pipeRadius).plugTo(pipeRadius).onChange(cb);
          
      cp5.addSlider("pipeLength").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(viewport_w/10, 0.7*viewport_w).setValue(pipeLength).plugTo(pipeLength).onChange(cb);
     
     cp5.addSlider("xpos").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
            .setRange(0, viewport_w).setValue(xpos).plugTo(xpos);
     
     cp5.addSlider("ypos").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
             .setRange(0, viewport_h).setValue(ypos).plugTo(ypos);
                               
     cp5.addSlider("vx").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
             .setRange(0, 1000).setValue(cb_fluid_data.vx).plugTo(cb_fluid_data, "vx").onChange(cb);
             
      cp5.addSlider("temperature").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 1).setValue(fluid.param.dissipation_temperature).plugTo(fluid.param, "dissipation_temperature");
          
      cp5.addSlider("P1").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 1).setValue(P1).plugTo(P1).onChange(cb);
          
       cp5.addSlider("P2").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 1).setValue(P2).plugTo(P2).onChange(cb);
          
  
/*cp5.addSlider("vorticity").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 1).setValue(fluid.param.vorticity).plugTo(fluid.param, "vorticity");
          
      cp5.addSlider("iterations").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 80).setValue(fluid.param.num_jacobi_projection).plugTo(fluid.param, "num_jacobi_projection");
            
      cp5.addSlider("timestep").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 1).setValue(fluid.param.timestep).plugTo(fluid.param, "timestep");
          
      cp5.addSlider("gridscale").setGroup(group_fluid).setSize(sx, sy).setPosition(px, py+=oy)
          .setRange(0, 50).setValue(fluid.param.gridscale).plugTo(fluid.param, "gridscale");*/
      
      RadioButton rb_setFluid_DisplayMode = cp5.addRadio("fluid_displayMode").setGroup(group_fluid).setSize(80,18).setPosition(px, py+=(int)(oy*1.5f))
          .setSpacingColumn(2).setSpacingRow(2).setItemsPerRow(2)
          .addItem("Temperature", 1)
          .addItem("Pressure"   ,2)
          .addItem("Velocity"   ,3)
          .activate(DISPLAY_fluid_texture_mode);
      for(Toggle toggle : rb_setFluid_DisplayMode.getItems()) toggle.getCaptionLabel().alignX(CENTER);
      
      cp5.addRadio("fluid_displayVelocityVectors").setGroup(group_fluid).setSize(18,18).setPosition(px, py+=(int)(oy*2.5f))
          .setSpacingColumn(2).setSpacingRow(2).setItemsPerRow(1)
          .addItem("Velocity Vectors", 0)
          .activate(DISPLAY_FLUID_VECTORS ? 0 : 2);
    }
   
    
    ////////////////////////////////////////////////////////////////////////////
    // GUI - DISPLAY
    ////////////////////////////////////////////////////////////////////////////
    /*Group group_display = cp5.addGroup("display");
    {
      group_display.setHeight(20).setSize(gui_w, 50)
      .setBackgroundColor(color(16, 180)).setColorBackground(color(16, 180));
      group_display.getCaptionLabel().align(CENTER, CENTER);
      
      px = 10; py = 15;
      
      cp5.addSlider("BACKGROUND").setGroup(group_display).setSize(sx,sy).setPosition(px, py)
          .setRange(0, 255).setValue(BACKGROUND_COLOR).plugTo(this, "BACKGROUND_COLOR");
    }*/
    
    
    ////////////////////////////////////////////////////////////////////////////
    // GUI - ACCORDION
    ////////////////////////////////////////////////////////////////////////////
    cp5.addAccordion("acc").setPosition(gui_x, gui_y).setWidth(gui_w).setSize(gui_w, height)
      .setCollapseMode(Accordion.MULTI)
      .addItem(group_fluid)
      .open(4);
  }
  

  
  public class ObstaclePainter{
    
    // 0 ... not drawing
    // 1 ... adding obstacles
    // 2 ... removing obstacles
    public int draw_mode = 0;
    PGraphics pg;
    
    float size_paint = 15;
    float size_clear = size_paint * 2.5f;
    
    float paint_x, paint_y;
    float clear_x, clear_y;
    
    int shading = 64;
    
    public ObstaclePainter(PGraphics pg){
      this.pg = pg;
    }
    
    public void beginDraw(int mode){
      paint_x = mouseX;
      paint_y = mouseY;
      this.draw_mode = mode;
      if(mode == 1){
        pg.beginDraw();
        pg.blendMode(REPLACE);
        pg.noStroke();
        pg.fill(shading);
        pg.ellipse(mouseX, mouseY, size_paint, size_paint);
        pg.endDraw();
      }
      if(mode == 2){
        clear(mouseX, mouseY);
      }
    }
    
    public boolean isDrawing(){
      return draw_mode != 0;
    }
    
    public void draw(){
      paint_x = mouseX;
      paint_y = mouseY;
      if(draw_mode == 1){
        pg.beginDraw();
        pg.blendMode(REPLACE);
        pg.strokeWeight(size_paint);
        pg.stroke(shading);
        pg.line(mouseX, mouseY, pmouseX, pmouseY);
        pg.endDraw();
      }
      if(draw_mode == 2){
        clear(mouseX, mouseY);
      }
      

      
    }

    public void endDraw(){
      this.draw_mode = 0;
    }
    
    public void clear(float x, float y){
      clear_x = x;
      clear_y = y;
      pg.beginDraw();
      pg.blendMode(REPLACE);
      pg.noStroke();
      pg.fill(0, 0);
      pg.ellipse(x, y, size_clear, size_clear);
      pg.endDraw();
    }
    
    public void displayBrush(PGraphics dst){
      if(draw_mode == 1){
        dst.strokeWeight(1);
        dst.stroke(0);
        dst.fill(200,50);
        dst.ellipse(paint_x, paint_y, size_paint, size_paint);
      }
      if(draw_mode == 2){
        dst.strokeWeight(1);
        dst.stroke(200);
        dst.fill(200,100);
        dst.ellipse(clear_x, clear_y, size_clear, size_clear);
      }
    }
    

  }
  
  
  
  