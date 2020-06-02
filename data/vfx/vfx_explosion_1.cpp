#include <core/unigine.h>
#include <world_selector.h>

WorldSelector selector;
/*
 */
int init() {
	
	selector = new WorldSelector("vfx/");
	Player camera = node_cast(engine.world.getNodeByName("camera"));
	engine.game.setPlayer(camera);
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
	return 1;
}
