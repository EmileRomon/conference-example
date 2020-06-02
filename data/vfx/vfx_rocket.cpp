#include <core/unigine.h>
#include <world_selector.h>
#include <core/systems/tracker/tracker.h>

using Unigine::Tracker;

/*
 */
Tracker tracker;
TrackerTrack track;
float time;
float min_time;
float max_time;
float unit_time;
int playing = 1;

WidgetHBox buttons;
WidgetIcon toggle_animation;
WidgetIcon reset_animation;
WorldSelector selector;

/*
 */
int init() {
	
	selector = new WorldSelector("vfx/");
	
	Player camera = node_cast(engine.world.getNodeByName("camera"));
	engine.game.setPlayer(camera);
	
	tracker = new Tracker(TRACKER_CHECK_OBJECTS);
	track = tracker.loadTrack("vfx/tracks/vfx_rocket.track");
	
	time = track.getMinTime();
	min_time = track.getMinTime();
	max_time = track.getMaxTime();
	unit_time = track.getUnitTime();
	
	
	Gui gui = engine.getGui();
	buttons = new WidgetHBox(gui,4,0);
	buttons.setPosition(100,100);
	gui.addChild(buttons,GUI_ALIGN_OVERLAP);
	
	reset_animation = new WidgetIcon(gui);
	reset_animation.setCallback(GUI_CLICKED,"reset_clicked");
	reset_animation.setTexture("core/systems/tracker/editor/images/tracker_time_prev.png");
	buttons.addChild(reset_animation,GUI_ALIGN_EXPAND);
	
	toggle_animation = new WidgetIcon(gui);
	toggle_animation.setTexture("core/systems/tracker/editor/images/tracker_time_play.png");
	toggle_animation.setToggleable(1);
	toggle_animation.setToggled(1);
	toggle_animation.setCallback(GUI_CLICKED,"toggle_clicked");
	buttons.addChild(toggle_animation,GUI_ALIGN_EXPAND);
	
	engine.game.setEnabled(playing);
	return 1;
}

/*
 */
int shutdown() {
	return 1;
}

/*
 */
int update() {
	selector.update();
	if(!playing) return 1;
	
	// update time
	time += engine.game.getIFps() / unit_time;
	time = min_time + ((time - min_time)) % (max_time - min_time);
	if(time > max_time) time = min_time;
	
	// set track
	if(engine.game.isEnabled())
		track.set(time);
	
	return 1;
}

void toggle_clicked() {
	playing = (playing) ? 0 : 1;
	engine.game.setEnabled(playing);
}

void reset_clicked() {
	time = min_time;
}
