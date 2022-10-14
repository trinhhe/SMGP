#ifndef _CALLBACKS_HPP__
#define _CALLBACKS_HPP__

#include <igl/opengl/glfw/Viewer.h>

using Viewer = igl::opengl::glfw::Viewer;

void callback_draw_viewer_menu();
void callback_draw_custom_window();

bool callback_pre_draw(Viewer &viewer);
bool callback_post_draw(Viewer &viewe);
bool callback_mouse_down(Viewer &viewer, int button, int modifier);
bool callback_mouse_up(Viewer &viewer, int button, int modifier);
bool callback_mouse_move(Viewer &viewer, int mouse_x, int mouse_y);
bool callback_mouse_scroll(Viewer &viewer, float delta_y);
bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers);
bool callback_key_up(Viewer &viewer, unsigned char key, int modifiers);

#endif // _CALLBACKS_HPP__
