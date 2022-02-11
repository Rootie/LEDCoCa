import glob
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import os
import platform
import sys
import math

isMacOS = (platform.system() == "Darwin")


class Settings:
    UNLIT = "defaultUnlit"

    def __init__(self):
        self.mouse_model = gui.SceneWidget.Controls.ROTATE_CAMERA
        self.bg_color = gui.Color(1, 1, 1)
        self.show_axes = True

        self.apply_material = True  # clear to False after processing
        self._materials = {
            Settings.UNLIT: rendering.MaterialRecord(),
        }
        self._materials[Settings.UNLIT].base_color = [1, 1, 1, 1.0]
        self._materials[Settings.UNLIT].shader = Settings.UNLIT

        # Conveniently, assigning from self._materials[...] assigns a reference,
        # not a copy, so if we change the property of a material, then switch
        # to another one, then come back, the old setting will still be there.
        self.material = self._materials[Settings.UNLIT]

    def set_material(self, name):
        self.material = self._materials[name]
        self.apply_material = True

    def apply_material_prefab(self, name):
        assert (self.material.shader == Settings.LIT)
        prefab = Settings.PREFAB[name]
        for key, val in prefab.items():
            setattr(self.material, "base_" + key, val)

    def apply_lighting_profile(self, name):
        profile = Settings.LIGHTING_PROFILES[name]
        for key, val in profile.items():
            setattr(self, key, val)


class AppWindow:
    MENU_QUIT = 3
    MENU_ABOUT = 21

    def __init__(self, width, height):
        self.settings = Settings()

        self.rot_x = 0
        self.rot_y = 0
        self.rot_z = 0

        self.window = gui.Application.instance.create_window(
            "LED CoCa", width, height)
        w = self.window  # to make the code more concise

        # 3D widget
        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(w.renderer)

        # ---- Settings panel ----
        # Rather than specifying sizes in pixels, which may vary in size based
        # on the monitor, especially on macOS which has 220 dpi monitors, use
        # the em-size. This way sizings will be proportional to the font size,
        # which will create a more visually consistent size across platforms.
        em = w.theme.font_size
        separation_height = int(round(0.5 * em))

        self._settings_panel = gui.Vert(
            0, gui.Margins(0.25 * em, 0.25 * em, 0.25 * em, 0.25 * em))

        rotations = gui.CollapsableVert("Model rotation", 0,
                                                gui.Margins(em, 0, 0, 0))

        self._rot_x_slider = gui.Slider(gui.Slider.INT)
        self._rot_x_slider.set_limits(0, 360)
        self._rot_x_slider.set_on_value_changed(self._on_rot_x)

        self._rot_y_slider = gui.Slider(gui.Slider.INT)
        self._rot_y_slider.set_limits(0, 360)
        self._rot_y_slider.set_on_value_changed(self._on_rot_y)

        self._rot_z_slider = gui.Slider(gui.Slider.INT)
        self._rot_z_slider.set_limits(0, 360)
        self._rot_z_slider.set_on_value_changed(self._on_rot_z)

        grid = gui.VGrid(2, 0.25 * em)
        grid.add_child(gui.Label("X"))
        grid.add_child(self._rot_x_slider)
        grid.add_child(gui.Label("Y"))
        grid.add_child(self._rot_y_slider)
        grid.add_child(gui.Label("Z"))
        grid.add_child(self._rot_z_slider)
        rotations.add_child(grid)

        self._settings_panel.add_child(rotations)

        view_ctrls = gui.CollapsableVert("View controls", 0.25 * em,
                                         gui.Margins(em, 0, 0, 0))

        self._arcball_button = gui.Button("Arcball")
        self._arcball_button.horizontal_padding_em = 0.5
        self._arcball_button.vertical_padding_em = 0
        self._arcball_button.set_on_clicked(self._set_mouse_mode_rotate)
        self._fly_button = gui.Button("Fly")
        self._fly_button.horizontal_padding_em = 0.5
        self._fly_button.vertical_padding_em = 0
        self._fly_button.set_on_clicked(self._set_mouse_mode_fly)
        view_ctrls.add_child(gui.Label("Mouse controls"))
        h = gui.Horiz(0.25 * em)
        h.add_stretch()
        h.add_child(self._arcball_button)
        h.add_child(self._fly_button)
        h.add_stretch()
        view_ctrls.add_child(h)

        self._bg_color = gui.ColorEdit()
        self._bg_color.set_on_value_changed(self._on_bg_color)

        grid = gui.VGrid(2, 0.25 * em)
        grid.add_child(gui.Label("BG Color"))
        grid.add_child(self._bg_color)
        view_ctrls.add_child(grid)

        self._show_axes = gui.Checkbox("Show axes")
        self._show_axes.set_on_checked(self._on_show_axes)
        view_ctrls.add_fixed(separation_height)
        view_ctrls.add_child(self._show_axes)

        self._settings_panel.add_child(view_ctrls)

        material_settings = gui.CollapsableVert("Material settings", 0,
                                                gui.Margins(em, 0, 0, 0))

        self._material_color = gui.ColorEdit()
        self._material_color.set_on_value_changed(self._on_material_color)
        self._point_size = gui.Slider(gui.Slider.INT)
        self._point_size.set_limits(1, 10)
        self._point_size.set_on_value_changed(self._on_point_size)

        grid = gui.VGrid(2, 0.25 * em)
        grid.add_child(gui.Label("Color"))
        grid.add_child(self._material_color)
        grid.add_child(gui.Label("Point size"))
        grid.add_child(self._point_size)
        material_settings.add_child(grid)

        self._settings_panel.add_fixed(separation_height)
        self._settings_panel.add_child(material_settings)
        # ----

        # Normally our user interface can be children of all one layout (usually
        # a vertical layout), which is then the only child of the window. In our
        # case we want the scene to take up all the space and the settings panel
        # to go above it. We can do this custom layout by providing an on_layout
        # callback. The on_layout callback should set the frame
        # (position + size) of every child correctly. After the callback is
        # done the window will layout the grandchildren.
        w.set_on_layout(self._on_layout)
        w.add_child(self._scene)
        w.add_child(self._settings_panel)

        # ---- Menu ----
        # The menu is global (because the macOS menu is global), so only create
        # it once, no matter how many windows are created
        if gui.Application.instance.menubar is None:
            if isMacOS:
                app_menu = gui.Menu()
                app_menu.add_item("About", AppWindow.MENU_ABOUT)
                app_menu.add_separator()
                app_menu.add_item("Quit", AppWindow.MENU_QUIT)
            file_menu = gui.Menu()
            if not isMacOS:
                #file_menu.add_separator()
                file_menu.add_item("Quit", AppWindow.MENU_QUIT)
            help_menu = gui.Menu()
            help_menu.add_item("About", AppWindow.MENU_ABOUT)

            menu = gui.Menu()
            if isMacOS:
                # macOS will name the first menu item for the running application
                # (in our case, probably "Python"), regardless of what we call
                # it. This is the application menu, and it is where the
                # About..., Preferences..., and Quit menu items typically go.
                menu.add_menu("Example", app_menu)
                menu.add_menu("File", file_menu)
                # Don't include help menu unless it has something more than
                # About...
            else:
                menu.add_menu("File", file_menu)
                menu.add_menu("Help", help_menu)
            gui.Application.instance.menubar = menu

        # The menubar is global, but we need to connect the menu items to the
        # window, so that the window can call the appropriate function when the
        # menu item is activated.
        w.set_on_menu_item_activated(AppWindow.MENU_QUIT, self._on_menu_quit)
        w.set_on_menu_item_activated(AppWindow.MENU_ABOUT, self._on_menu_about)
        # ----

        self._apply_settings()

    def _apply_settings(self):
        bg_color = [
            self.settings.bg_color.red, self.settings.bg_color.green,
            self.settings.bg_color.blue, self.settings.bg_color.alpha
        ]
        self._scene.scene.set_background(bg_color)
        self._scene.scene.show_skybox(False)
        self._scene.scene.show_axes(self.settings.show_axes)

        if self.settings.apply_material:
            self._scene.scene.update_material(self.settings.material)
            self.settings.apply_material = False

        self._bg_color.color_value = self.settings.bg_color
        self._show_axes.checked = self.settings.show_axes
        c = gui.Color(self.settings.material.base_color[0],
                      self.settings.material.base_color[1],
                      self.settings.material.base_color[2],
                      self.settings.material.base_color[3])
        self._material_color.color_value = c
        self._point_size.double_value = self.settings.material.point_size

    def _on_layout(self, layout_context):
        # The on_layout callback should set the frame (position + size) of every
        # child correctly. After the callback is done the window will layout
        # the grandchildren.
        r = self.window.content_rect
        self._scene.frame = r
        width = 17 * layout_context.theme.font_size
        height = min(
            r.height,
            self._settings_panel.calc_preferred_size(
                layout_context, gui.Widget.Constraints()).height)
        self._settings_panel.frame = gui.Rect(r.get_right() - width, r.y, width,
                                              height)

    def _set_mouse_mode_rotate(self):
        self._scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

    def _set_mouse_mode_fly(self):
        self._scene.set_view_controls(gui.SceneWidget.Controls.FLY)

    def _on_bg_color(self, new_color):
        self.settings.bg_color = new_color
        self._apply_settings()

    def _on_show_axes(self, show):
        self.settings.show_axes = show
        self._apply_settings()


    def _on_material_color(self, color):
        self.settings.material.base_color = [
            color.red, color.green, color.blue, color.alpha
        ]
        self.settings.apply_material = True
        self._apply_settings()

    def _update_rotation(self):
        R = np.matrix([
            [ math.cos(self.rot_y) * math.cos(self.rot_z), -math.sin(self.rot_z), math.sin(self.rot_y)],
            [ math.sin(self.rot_z), math.cos(self.rot_x) * math.cos(self.rot_z), -math.sin(self.rot_x)],
            [ -math.sin(self.rot_y), math.sin(self.rot_x), math.cos(self.rot_x) * math.cos(self.rot_y)]])
        new_points = np.array(self._points_3D * R)
        geometry = o3d.geometry.PointCloud()
        geometry.points = o3d.utility.Vector3dVector(new_points)
        self._geometry = geometry

        self._scene.scene.clear_geometry()
        self._scene.scene.add_geometry("__model__", geometry,
                                        self.settings.material)
        lines = []
        colors = []
        for i in range(len(geometry.points) - 1):
            lines.append([i, i+1])
            colors.append([i / len(geometry.points), 1-(i / len(geometry.points)), 0])
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(geometry.points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)
        self._scene.scene.add_geometry("__lines__", line_set,
                                        self.settings.material)


    def _on_rot_x(self, rot_x):
        self.rot_x = rot_x * math.pi / 180
        self._update_rotation()

    def _on_rot_y(self, rot_y):
        self.rot_y = rot_y * math.pi / 180
        self._update_rotation()
    
    def _on_rot_z(self, rot_z):
        self.rot_z = rot_z * math.pi / 180
        self._update_rotation()
    
    def _on_point_size(self, size):
        self.settings.material.point_size = int(size)
        self.settings.apply_material = True
        self._apply_settings()

    def _on_menu_quit(self):
        gui.Application.instance.quit()

    def _on_menu_about(self):
        em = self.window.theme.font_size
        dlg = gui.Dialog("About")

        dlg_layout = gui.Vert(em, gui.Margins(em, em, em, em))
        dlg_layout.add_child(gui.Label("LED CoCa 0.1"))

        ok = gui.Button("OK")
        ok.set_on_clicked(self._on_about_ok)

        h = gui.Horiz()
        h.add_stretch()
        h.add_child(ok)
        h.add_stretch()
        dlg_layout.add_child(h)

        dlg.add_child(dlg_layout)
        self.window.show_dialog(dlg)

    def _on_about_ok(self):
        self.window.close_dialog()

    def set_points(self, points_3D):
        self._points_3D = points_3D
        try:
            self._update_rotation()
            bounds = self._geometry.get_axis_aligned_bounding_box()
            self._scene.setup_camera(60, bounds, bounds.get_center())
        except Exception as e:
            print(e)
