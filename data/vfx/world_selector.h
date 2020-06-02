
class WorldSelector {
	
	string path;
	Gui gui;
	WidgetComboBox combobox;
	WidgetVBox layout;

	float refresh_time;
	float last_time;
	int nodes[0];
	
	WorldSelector(string p) {
		
		gui = engine.getGui();
		
		layout = new WidgetVBox(gui,4,0);
		layout.setPosition(100,50);
		
		combobox = new WidgetComboBox(gui);
		layout.addChild(new WidgetLabel(gui,"Choose sample"),GUI_ALIGN_EXPAND);
		layout.addChild(combobox,GUI_ALIGN_EXPAND);
		
		layout.arrange();
		gui.addChild(layout,GUI_ALIGN_OVERLAP);
		
		string current_world = basename(replace(engine.world.getPath(),".world",""));
		path = engine.filesystem.resolvePartialVirtualPath(p);
		
		string files[0];
		engine.filesystem.getVirtualFiles(files);
		files.sort();
		foreach(string file; files) {
			if(strstr(file, path) == 0 && extension(file) == "world") {
				string name = extension(basename(file),"");
				combobox.addItem(name);
				if(strcmp(name,current_world) == 0) {
					combobox.setCurrentItem(combobox.getNumItems() - 1);
				}
			}
		}
		
		combobox.setCallback(GUI_CHANGED,"WorldSelector::world_changed_redirector",this);
		
		get_nodes();
	}
	
	void world_changed_redirector(WorldSelector w) {
		w.call("world_changed");
	}
	
	void world_changed() {
		engine.console.run("world_load " + combobox.getCurrentItemText());
		get_nodes();
	}

	void get_nodes() {
		refresh_time = -1;
		nodes.clear();
		engine.world.getNodes(nodes);
		get_node_refresh_time();
		last_time = engine.game.getTime();
	}

	void refresh_nodes() {
		foreach(Node node; nodes) {
			node.setEnabled(false);
			node.setEnabled(true);
		}
	}

	void get_node_refresh_time() {
		//remove node from nodes if property refresh_time not founded. Get refresh_time from node.
		for(int i = 0; i < nodes.size(); i++) {
			Node node = nodes[i];
			if(!node.getNumProperties()) {
				nodes.remove(i);
				i--;
				continue;
			}
			for(int i = 0; i < node.getNumProperties(); i++) {
				Property prop = node.getProperty(i);
				PropertyParameter pp = prop.getParameterPtr();
				int index = pp.findChild("time");
				if(index == -1) {
					nodes.remove(i);
					i--;
					continue;
				}
				refresh_time = pp.getChild(index).getValueFloat();
			}
		}
	}

	void update() {
		if(refresh_time == -1) return;
		if((engine.game.getTime() - last_time) >= refresh_time) {
			last_time = engine.game.getTime();
			refresh_nodes();
		}
	}

};
