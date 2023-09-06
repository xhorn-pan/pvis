#include "core/lv_disp.h"
#include "misc/lv_area.h"
#include "widgets/lv_btn.h"
#include "widgets/lv_label.h"
void pvis_ui(void) {
  lv_obj_t *btn;
  lv_obj_t *lbl;
  btn = lv_btn_create(lv_scr_act());

  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 20);

  lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "PVIS");
}
