##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.4.0-B60] date: [Fri Aug 09 00:10:14 CEST 2024] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
# \ 2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = reverse_mono_6ks


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
LVGL_DIR ?= .
LVGL_DIR_NAME ?= lvgl

######################################
# source
######################################
# C sources
C_SOURCES = \
Src/main.c \
src/ui/screens.c \
src/ui/images.c \
src/ui/styles.c \
src/ui/ui.c \
src/ui/eez-flow-lz4.c \
src/ui/eez-flow-sha256.c \
lvgl/examples/libs/ffmpeg/lv_example_ffmpeg_2.c \
lvgl/examples/libs/ffmpeg/lv_example_ffmpeg_1.c \
lvgl/examples/libs/qrcode/lv_example_qrcode_1.c \
lvgl/examples/libs/bmp/lv_example_bmp_1.c \
lvgl/examples/libs/freetype/lv_example_freetype_1.c \
lvgl/examples/libs/rlottie/lv_example_rlottie_1.c \
lvgl/examples/libs/rlottie/lv_example_rlottie_2.c \
lvgl/examples/libs/rlottie/lv_example_rlottie_approve.c \
lvgl/examples/libs/sjpg/lv_example_sjpg_1.c \
lvgl/examples/libs/png/img_wink_png.c \
lvgl/examples/libs/png/lv_example_png_1.c \
lvgl/examples/libs/gif/img_bulb_gif.c \
lvgl/examples/libs/gif/lv_example_gif_1.c \
lvgl/examples/styles/lv_example_style_4.c \
lvgl/examples/styles/lv_example_style_10.c \
lvgl/examples/styles/lv_example_style_15.c \
lvgl/examples/styles/lv_example_style_14.c \
lvgl/examples/styles/lv_example_style_9.c \
lvgl/examples/styles/lv_example_style_8.c \
lvgl/examples/styles/lv_example_style_2.c \
lvgl/examples/styles/lv_example_style_5.c \
lvgl/examples/styles/lv_example_style_11.c \
lvgl/examples/styles/lv_example_style_7.c \
lvgl/examples/styles/lv_example_style_6.c \
lvgl/examples/styles/lv_example_style_3.c \
lvgl/examples/styles/lv_example_style_1.c \
lvgl/examples/styles/lv_example_style_12.c \
lvgl/examples/styles/lv_example_style_13.c \
lvgl/examples/get_started/lv_example_get_started_3.c \
lvgl/examples/get_started/lv_example_get_started_1.c \
lvgl/examples/get_started/lv_example_get_started_2.c \
lvgl/examples/scroll/lv_example_scroll_1.c \
lvgl/examples/scroll/lv_example_scroll_2.c \
lvgl/examples/scroll/lv_example_scroll_5.c \
lvgl/examples/scroll/lv_example_scroll_6.c \
lvgl/examples/scroll/lv_example_scroll_3.c \
lvgl/examples/scroll/lv_example_scroll_4.c \
lvgl/examples/anim/lv_example_anim_timeline_1.c \
lvgl/examples/anim/lv_example_anim_1.c \
lvgl/examples/anim/lv_example_anim_2.c \
lvgl/examples/anim/lv_example_anim_3.c \
lvgl/examples/porting/lv_port_disp_template.c \
lvgl/examples/porting/lv_port_indev_template.c \
lvgl/examples/porting/lv_port_fs_template.c \
lvgl/examples/others/monkey/lv_example_monkey_2.c \
lvgl/examples/others/monkey/lv_example_monkey_1.c \
lvgl/examples/others/monkey/lv_example_monkey_3.c \
lvgl/examples/others/msg/lv_example_msg_3.c \
lvgl/examples/others/msg/lv_example_msg_2.c \
lvgl/examples/others/msg/lv_example_msg_1.c \
lvgl/examples/others/snapshot/lv_example_snapshot_1.c \
lvgl/examples/others/gridnav/lv_example_gridnav_3.c \
lvgl/examples/others/gridnav/lv_example_gridnav_2.c \
lvgl/examples/others/gridnav/lv_example_gridnav_4.c \
lvgl/examples/others/gridnav/lv_example_gridnav_1.c \
lvgl/examples/others/imgfont/lv_example_imgfont_1.c \
lvgl/examples/others/fragment/lv_example_fragment_1.c \
lvgl/examples/others/fragment/lv_example_fragment_2.c \
lvgl/examples/others/ime/lv_example_ime_pinyin_1.c \
lvgl/examples/others/ime/lv_example_ime_pinyin_2.c \
lvgl/examples/assets/img_cogwheel_alpha16.c \
lvgl/examples/assets/img_cogwheel_indexed16.c \
lvgl/examples/assets/img_cogwheel_argb.c \
lvgl/examples/assets/img_hand.c \
lvgl/examples/assets/imgbtn_right.c \
lvgl/examples/assets/img_skew_strip.c \
lvgl/examples/assets/imgbtn_mid.c \
lvgl/examples/assets/img_cogwheel_rgb.c \
lvgl/examples/assets/imgbtn_left.c \
lvgl/examples/assets/img_cogwheel_chroma_keyed.c \
lvgl/examples/assets/img_caret_down.c \
lvgl/examples/assets/img_star.c \
lvgl/examples/assets/animimg002.c \
lvgl/examples/assets/animimg001.c \
lvgl/examples/assets/emoji/img_emoji_F617.c \
lvgl/examples/assets/animimg003.c \
lvgl/examples/layouts/flex/lv_example_flex_2.c \
lvgl/examples/layouts/flex/lv_example_flex_1.c \
lvgl/examples/layouts/flex/lv_example_flex_5.c \
lvgl/examples/layouts/flex/lv_example_flex_6.c \
lvgl/examples/layouts/flex/lv_example_flex_3.c \
lvgl/examples/layouts/flex/lv_example_flex_4.c \
lvgl/examples/layouts/grid/lv_example_grid_6.c \
lvgl/examples/layouts/grid/lv_example_grid_4.c \
lvgl/examples/layouts/grid/lv_example_grid_3.c \
lvgl/examples/layouts/grid/lv_example_grid_1.c \
lvgl/examples/layouts/grid/lv_example_grid_5.c \
lvgl/examples/layouts/grid/lv_example_grid_2.c \
lvgl/examples/widgets/table/lv_example_table_1.c \
lvgl/examples/widgets/table/lv_example_table_2.c \
lvgl/examples/widgets/dropdown/lv_example_dropdown_3.c \
lvgl/examples/widgets/dropdown/lv_example_dropdown_1.c \
lvgl/examples/widgets/dropdown/lv_example_dropdown_2.c \
lvgl/examples/widgets/calendar/lv_example_calendar_1.c \
lvgl/examples/widgets/animimg/lv_example_animimg_1.c \
lvgl/examples/widgets/label/lv_example_label_1.c \
lvgl/examples/widgets/label/lv_example_label_3.c \
lvgl/examples/widgets/label/lv_example_label_4.c \
lvgl/examples/widgets/label/lv_example_label_5.c \
lvgl/examples/widgets/label/lv_example_label_2.c \
lvgl/examples/widgets/roller/lv_example_roller_2.c \
lvgl/examples/widgets/roller/lv_example_roller_3.c \
lvgl/examples/widgets/roller/lv_example_roller_1.c \
lvgl/examples/widgets/checkbox/lv_example_checkbox_2.c \
lvgl/examples/widgets/checkbox/lv_example_checkbox_1.c \
lvgl/examples/widgets/bar/lv_example_bar_2.c \
lvgl/examples/widgets/bar/lv_example_bar_4.c \
lvgl/examples/widgets/bar/lv_example_bar_3.c \
lvgl/examples/widgets/bar/lv_example_bar_5.c \
lvgl/examples/widgets/bar/lv_example_bar_1.c \
lvgl/examples/widgets/bar/lv_example_bar_6.c \
lvgl/examples/widgets/list/lv_example_list_2.c \
lvgl/examples/widgets/list/lv_example_list_1.c \
lvgl/examples/widgets/imgbtn/lv_example_imgbtn_1.c \
lvgl/examples/widgets/img/lv_example_img_3.c \
lvgl/examples/widgets/img/lv_example_img_4.c \
lvgl/examples/widgets/img/lv_example_img_1.c \
lvgl/examples/widgets/img/lv_example_img_2.c \
lvgl/examples/widgets/msgbox/lv_example_msgbox_1.c \
lvgl/examples/widgets/btnmatrix/lv_example_btnmatrix_3.c \
lvgl/examples/widgets/btnmatrix/lv_example_btnmatrix_2.c \
lvgl/examples/widgets/btnmatrix/lv_example_btnmatrix_1.c \
lvgl/examples/widgets/obj/lv_example_obj_1.c \
lvgl/examples/widgets/obj/lv_example_obj_2.c \
lvgl/examples/widgets/tileview/lv_example_tileview_1.c \
lvgl/examples/widgets/win/lv_example_win_1.c \
lvgl/src/extra/widgets/win/lv_win.c \
lvgl/examples/widgets/tabview/lv_example_tabview_1.c \
lvgl/examples/widgets/tabview/lv_example_tabview_2.c \
lvgl/examples/widgets/canvas/lv_example_canvas_1.c \
lvgl/examples/widgets/canvas/lv_example_canvas_2.c \
lvgl/examples/widgets/slider/lv_example_slider_3.c \
lvgl/examples/widgets/slider/lv_example_slider_2.c \
lvgl/examples/widgets/slider/lv_example_slider_1.c \
lvgl/examples/widgets/arc/lv_example_arc_1.c \
lvgl/examples/widgets/arc/lv_example_arc_2.c \
lvgl/examples/widgets/switch/lv_example_switch_1.c \
lvgl/examples/widgets/chart/lv_example_chart_9.c \
lvgl/examples/widgets/chart/lv_example_chart_3.c \
lvgl/examples/widgets/chart/lv_example_chart_2.c \
lvgl/examples/widgets/chart/lv_example_chart_1.c \
lvgl/examples/widgets/chart/lv_example_chart_8.c \
lvgl/examples/widgets/chart/lv_example_chart_7.c \
lvgl/examples/widgets/chart/lv_example_chart_5.c \
lvgl/examples/widgets/chart/lv_example_chart_4.c \
lvgl/examples/widgets/chart/lv_example_chart_6.c \
lvgl/examples/widgets/keyboard/lv_example_keyboard_1.c \
lvgl/examples/widgets/btn/lv_example_btn_2.c \
lvgl/examples/widgets/btn/lv_example_btn_3.c \
lvgl/examples/widgets/btn/lv_example_btn_1.c \
lvgl/examples/widgets/menu/lv_example_menu_1.c \
lvgl/examples/widgets/menu/lv_example_menu_3.c \
lvgl/examples/widgets/menu/lv_example_menu_4.c \
lvgl/examples/widgets/menu/lv_example_menu_2.c \
lvgl/examples/widgets/menu/lv_example_menu_5.c \
lvgl/examples/widgets/textarea/lv_example_textarea_2.c \
lvgl/examples/widgets/textarea/lv_example_textarea_1.c \
lvgl/examples/widgets/textarea/lv_example_textarea_3.c \
lvgl/examples/widgets/span/lv_example_span_1.c \
lvgl/examples/widgets/spinbox/lv_example_spinbox_1.c \
lvgl/examples/widgets/spinner/lv_example_spinner_1.c \
lvgl/examples/widgets/led/lv_example_led_1.c \
lvgl/examples/widgets/colorwheel/lv_example_colorwheel_1.c \
lvgl/examples/widgets/line/lv_example_line_1.c \
lvgl/examples/widgets/meter/lv_example_meter_3.c \
lvgl/examples/widgets/meter/lv_example_meter_4.c \
lvgl/examples/widgets/meter/lv_example_meter_2.c \
lvgl/examples/widgets/meter/lv_example_meter_1.c \
lvgl/examples/event/lv_example_event_1.c \
lvgl/examples/event/lv_example_event_3.c \
lvgl/examples/event/lv_example_event_2.c \
lvgl/examples/event/lv_example_event_4.c \
lvgl/env_support/rt-thread/lv_rt_thread_port.c \
lvgl/env_support/rt-thread/squareline/lv_ui_entry.c \
lvgl/src/draw/lv_img_decoder.c \
lvgl/src/draw/sw/lv_draw_sw_letter.c \
lvgl/src/draw/sw/lv_draw_sw_polygon.c \
lvgl/src/draw/sw/lv_draw_sw_img.c \
lvgl/src/draw/sw/lv_draw_sw_transform.c \
lvgl/src/draw/sw/lv_draw_sw.c \
lvgl/src/draw/sw/lv_draw_sw_line.c \
lvgl/src/draw/sw/lv_draw_sw_blend.c \
lvgl/src/draw/sw/lv_draw_sw_rect.c \
lvgl/src/draw/sw/lv_draw_sw_gradient.c \
lvgl/src/draw/sw/lv_draw_sw_arc.c \
lvgl/src/draw/sw/lv_draw_sw_layer.c \
lvgl/src/draw/sw/lv_draw_sw_dither.c \
lvgl/src/draw/lv_draw_mask.c \
lvgl/src/draw/lv_draw_img.c \
lvgl/src/draw/lv_draw.c \
lvgl/src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.c \
lvgl/src/draw/lv_draw_triangle.c \
lvgl/src/draw/lv_draw_transform.c \
lvgl/src/draw/lv_img_cache.c \
lvgl/src/draw/lv_draw_layer.c \
lvgl/src/draw/lv_img_buf.c \
lvgl/src/draw/lv_draw_line.c \
lvgl/src/draw/lv_draw_rect.c \
lvgl/src/draw/lv_draw_label.c \
lvgl/src/draw/lv_draw_arc.c \
lvgl/src/core/lv_obj_draw.c \
lvgl/src/core/lv_event.c \
lvgl/src/core/lv_obj_style_gen.c \
lvgl/src/core/lv_obj_tree.c \
lvgl/src/core/lv_obj_style.c \
lvgl/src/core/lv_group.c \
lvgl/src/core/lv_obj_class.c \
lvgl/src/core/lv_indev.c \
lvgl/src/core/lv_obj_scroll.c \
lvgl/src/core/lv_obj.c \
lvgl/src/core/lv_indev_scroll.c \
lvgl/src/core/lv_refr.c \
lvgl/src/core/lv_disp.c \
lvgl/src/core/lv_theme.c \
lvgl/src/core/lv_obj_pos.c \
lvgl/src/font/lv_font_montserrat_8.c \
lvgl/src/font/lv_font_montserrat_38.c \
lvgl/src/font/lv_font_montserrat_16.c \
lvgl/src/font/lv_font_montserrat_32.c \
lvgl/src/font/lv_font_montserrat_18.c \
lvgl/src/font/lv_font_simsun_16_cjk.c \
lvgl/src/font/lv_font_montserrat_12_subpx.c \
lvgl/src/font/lv_font_montserrat_46.c \
lvgl/src/font/lv_font_montserrat_48.c \
lvgl/src/font/lv_font_dejavu_16_persian_hebrew.c \
lvgl/src/font/lv_font_unscii_8.c \
lvgl/src/font/lv_font_montserrat_24.c \
lvgl/src/font/lv_font_montserrat_10.c \
lvgl/src/font/lv_font.c \
lvgl/src/font/lv_font_montserrat_40.c \
lvgl/src/font/lv_font_montserrat_30.c \
lvgl/src/font/lv_font_montserrat_42.c \
lvgl/src/font/lv_font_loader.c \
lvgl/src/font/lv_font_montserrat_34.c \
lvgl/src/font/lv_font_montserrat_26.c \
lvgl/src/font/lv_font_montserrat_12.c \
lvgl/src/font/lv_font_montserrat_22.c \
lvgl/src/font/lv_font_montserrat_28.c \
lvgl/src/font/lv_font_montserrat_20.c \
lvgl/src/font/lv_font_montserrat_36.c \
lvgl/src/font/lv_font_unscii_16.c \
lvgl/src/font/lv_font_fmt_txt.c \
lvgl/src/font/lv_font_montserrat_14.c \
lvgl/src/font/lv_font_montserrat_28_compressed.c \
lvgl/src/font/lv_font_montserrat_44.c \
lvgl/src/hal/lv_hal_tick.c \
lvgl/src/hal/lv_hal_indev.c \
lvgl/src/hal/lv_hal_disp.c \
lvgl/src/extra/libs/ffmpeg/lv_ffmpeg.c \
lvgl/src/extra/libs/qrcode/lv_qrcode.c \
lvgl/src/extra/libs/qrcode/qrcodegen.c \
lvgl/src/extra/libs/bmp/lv_bmp.c \
lvgl/src/extra/libs/tiny_ttf/lv_tiny_ttf.c \
lvgl/src/extra/libs/freetype/lv_freetype.c \
lvgl/src/extra/libs/rlottie/lv_rlottie.c \
lvgl/src/extra/libs/sjpg/lv_sjpg.c \
lvgl/src/extra/libs/sjpg/tjpgd.c \
lvgl/src/extra/libs/png/lv_png.c \
lvgl/src/extra/libs/png/lodepng.c \
lvgl/src/extra/libs/gif/lv_gif.c \
lvgl/src/extra/libs/gif/gifdec.c \
lvgl/src/extra/libs/fsdrv/lv_fs_fatfs.c \
lvgl/src/extra/libs/fsdrv/lv_fs_littlefs.c \
lvgl/src/extra/libs/fsdrv/lv_fs_stdio.c \
lvgl/src/extra/themes/mono/lv_theme_mono.c \
lvgl/src/extra/themes/basic/lv_theme_basic.c \
lvgl/src/extra/themes/default/lv_theme_default.c \
lvgl/src/extra/others/monkey/lv_monkey.c \
lvgl/src/extra/others/msg/lv_msg.c \
lvgl/src/extra/others/snapshot/lv_snapshot.c \
lvgl/src/extra/others/gridnav/lv_gridnav.c \
lvgl/src/extra/others/imgfont/lv_imgfont.c \
lvgl/src/extra/others/fragment/lv_fragment_manager.c \
lvgl/src/extra/others/fragment/lv_fragment.c \
lvgl/src/extra/others/ime/lv_ime_pinyin.c \
lvgl/src/extra/lv_extra.c \
lvgl/src/extra/layouts/flex/lv_flex.c \
lvgl/src/extra/layouts/grid/lv_grid.c \
lvgl/src/extra/widgets/calendar/lv_calendar_header_arrow.c \
lvgl/src/extra/widgets/calendar/lv_calendar_header_dropdown.c \
lvgl/src/extra/widgets/calendar/lv_calendar.c \
lvgl/src/extra/widgets/animimg/lv_animimg.c \
lvgl/src/extra/widgets/list/lv_list.c \
lvgl/src/extra/widgets/imgbtn/lv_imgbtn.c \
lvgl/src/extra/widgets/msgbox/lv_msgbox.c \
lvgl/src/extra/widgets/tileview/lv_tileview.c \
lvgl/src/extra/widgets/tabview/lv_tabview.c \
lvgl/src/extra/widgets/chart/lv_chart.c \
lvgl/src/extra/widgets/keyboard/lv_keyboard.c \
lvgl/src/extra/widgets/menu/lv_menu.c \
lvgl/src/extra/widgets/span/lv_span.c \
lvgl/src/extra/widgets/spinbox/lv_spinbox.c \
lvgl/src/extra/widgets/spinner/lv_spinner.c \
lvgl/src/extra/widgets/led/lv_led.c \
lvgl/src/extra/widgets/colorwheel/lv_colorwheel.c \
lvgl/src/extra/widgets/meter/lv_meter.c \
lvgl/src/misc/lv_anim_timeline.c \
lvgl/src/misc/lv_ll.c \
lvgl/src/misc/lv_templ.c \
lvgl/src/misc/lv_area.c \
lvgl/src/misc/lv_txt_ap.c \
lvgl/src/misc/lv_anim.c \
lvgl/src/misc/lv_mem.c \
lvgl/src/misc/lv_txt.c \
lvgl/src/misc/lv_style.c \
lvgl/src/misc/lv_fs.c \
lvgl/src/misc/lv_lru.c \
lvgl/src/misc/lv_color.c \
lvgl/src/misc/lv_async.c \
lvgl/src/misc/lv_log.c \
lvgl/src/misc/lv_utils.c \
lvgl/src/misc/lv_style_gen.c \
lvgl/src/misc/lv_gc.c \
lvgl/src/misc/lv_printf.c \
lvgl/src/misc/lv_tlsf.c \
lvgl/src/misc/lv_timer.c \
lvgl/src/misc/lv_math.c \
lvgl/src/misc/lv_bidi.c \
lvgl/src/widgets/lv_arc.c \
lvgl/src/widgets/lv_objx_templ.c \
lvgl/src/widgets/lv_textarea.c \
lvgl/src/widgets/lv_line.c \
lvgl/src/widgets/lv_btn.c \
lvgl/src/widgets/lv_btnmatrix.c \
lvgl/src/widgets/lv_dropdown.c \
lvgl/src/widgets/lv_label.c \
lvgl/src/widgets/lv_slider.c \
lvgl/src/widgets/lv_bar.c \
lvgl/src/widgets/lv_img.c \
lvgl/src/widgets/lv_checkbox.c \
lvgl/src/widgets/lv_roller.c \
lvgl/src/widgets/lv_table.c \
lvgl/src/widgets/lv_switch.c \
lvgl/src/widgets/lv_canvas.c \
lvgl/demos/keypad_encoder/lv_demo_keypad_encoder.c \
lvgl/demos/stress/lv_demo_stress.c \
lvgl/demos/benchmark/lv_demo_benchmark.c \
lvgl/demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.c \
lvgl/demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.c \
lvgl/demos/benchmark/assets/img_benchmark_cogwheel_indexed16.c \
lvgl/demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.c \
lvgl/demos/benchmark/assets/img_benchmark_cogwheel_argb.c \
lvgl/demos/benchmark/assets/img_benchmark_cogwheel_alpha16.c \
lvgl/demos/benchmark/assets/img_benchmark_cogwheel_rgb.c \
lvgl/demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.c \
lvgl/demos/benchmark/assets/img_benchmark_cogwheel_rgb565a8.c \
lvgl/demos/music/lv_demo_music.c \
lvgl/demos/music/lv_demo_music_list.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_pause_large.c \
lvgl/demos/music/assets/img_lv_demo_music_slider_knob_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_loop_large.c \
lvgl/demos/music/assets/img_lv_demo_music_cover_1.c \
lvgl/demos/music/assets/img_lv_demo_music_wave_top.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_list_play_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_play.c \
lvgl/demos/music/assets/img_lv_demo_music_cover_3_large.c \
lvgl/demos/music/assets/img_lv_demo_music_cover_1_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_pause.c \
lvgl/demos/music/assets/img_lv_demo_music_wave_top_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_corner_large.c \
lvgl/demos/music/assets/img_lv_demo_music_cover_2.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_2_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_rnd_large.c \
lvgl/demos/music/assets/img_lv_demo_music_corner_left_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_list_pause_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_prev.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_rnd.c \
lvgl/demos/music/assets/img_lv_demo_music_list_border_large.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_1_large.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_2.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_prev_large.c \
lvgl/demos/music/assets/img_lv_demo_music_logo.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_play_large.c \
lvgl/demos/music/assets/img_lv_demo_music_cover_2_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_list_pause.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_4_large.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_4.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_loop.c \
lvgl/demos/music/assets/img_lv_demo_music_wave_bottom_large.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_3.c \
lvgl/demos/music/assets/img_lv_demo_music_cover_3.c \
lvgl/demos/music/assets/img_lv_demo_music_list_border.c \
lvgl/demos/music/assets/img_lv_demo_music_corner_left.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_next.c \
lvgl/demos/music/assets/img_lv_demo_music_wave_bottom.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_next_large.c \
lvgl/demos/music/assets/img_lv_demo_music_btn_list_play.c \
lvgl/demos/music/assets/img_lv_demo_music_corner_right_large.c \
lvgl/demos/music/assets/img_lv_demo_music_slider_knob.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_3_large.c \
lvgl/demos/music/assets/img_lv_demo_music_icon_1.c \
lvgl/demos/music/assets/img_lv_demo_music_corner_right.c \
lvgl/demos/music/lv_demo_music_main.c \
lvgl/demos/widgets/lv_demo_widgets.c \
lvgl/demos/widgets/assets/img_lvgl_logo.c \
lvgl/demos/widgets/assets/img_demo_widgets_avatar.c \
lvgl/demos/widgets/assets/img_clothes.c \
Drivers/LCD/ILI9341/ILI9341_GFX.c \
Drivers/LCD/ILI9341/ILI9341_STM32_Driver.c \
Drivers/TS/Src/XPT2046_lv.c \
Drivers/TS/Src/touchpad.c \
Src/stm32f4xx_it.c \
Src/stm32f4xx_hal_msp.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fmc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Src/system_stm32f4xx.c \
Src/sysmem.c \
Src/syscalls.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c
#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c#Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma2d.c
include lvgl/lvgl.mk

# ASM sources
ASM_SOURCES = \
startup_stm32f427xx.s

# C++ source files
CPP_SOURCES = \
    src/ui/eez-flow.cpp

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
CXX = $(GCC_PATH)/$(PREFIX)g++
SZ = $(GCC_PATH)/$(PREFIX)size
else
CXX = $(PREFIX)c++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F427xx \
-DLV_LVGL_H_INCLUDE_SIMPLE

CPP_DEFS = \
-DLV_LVGL_H_INCLUDE_SIMPLE

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES = \
-IInc \
-Ilvgl \
-Isrc/ui \
-IDrivers/TS/Inc \
-IDrivers/LCD/ILI9341 \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include

CPP_INCLUDES = \
-Ilvgl \
-Isrc/ui \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CPPFLAGS += $(MCU) $(CPP_DEFS) $(CPP_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -fcommon -MMD -MP -MF"$(@:%.o=%.d)"

# Generate dependency information
CPPFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F427ZGTx_FLASH.ld

# libraries
#LIBS = -lc -lm -lnosys 

LIBS = -lc -lm -lnosys -lgcc -lstdc++

LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################


#######################################
# list of objects
# Update the list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o))) \
          $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o))) \
          $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o))) \
          $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# Update the vpath for C++ files
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
#OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
#vpath %.s $(sort $(dir $(ASM_SOURCES)))
#OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
#vpath %.S $(sort $(dir $(ASMM_SOURCES)))

# Rule for building C++ files
$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(CPPFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
