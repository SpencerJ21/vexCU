#include "main.h"
#include "robot.hpp"

// modified version of https://github.com/theol0403/odomDebug

lv_style_t scrStyle;
lv_style_t fldStyle;
lv_style_t greyStyle;
lv_style_t ledStyle;
lv_style_t lineStyle;
lv_style_t targetStyle;
lv_style_t textStyle;

void screenTaskFn(){
  lv_obj_t *scr = lv_obj_create(NULL, NULL);
  lv_scr_load(scr);

  lv_style_copy(&scrStyle, &lv_style_plain_color);
  scrStyle.body.main_color = LV_COLOR_BLUE;
  scrStyle.body.grad_color = LV_COLOR_BLUE;
  scrStyle.body.border.width = 0;
  scrStyle.body.radius = 0;
  lv_obj_set_style(scr, &scrStyle);

  lv_obj_t* field = lv_obj_create(scr, NULL);
  lv_obj_set_size(field, 240, 240);
  lv_obj_align(field, NULL, LV_ALIGN_IN_RIGHT_MID, 0, 0);

  lv_style_copy(&fldStyle, &scrStyle);
  fldStyle.body.main_color = LV_COLOR_WHITE;
  fldStyle.body.grad_color = LV_COLOR_WHITE;
  lv_obj_set_style(field, &fldStyle);

  lv_style_copy(&greyStyle, &lv_style_plain);
  greyStyle.body.main_color = LV_COLOR_HEX(0x828F8F);
  greyStyle.body.grad_color = LV_COLOR_HEX(0x828F8F);
  greyStyle.body.border.width = 1;
  greyStyle.body.radius = 0;
  greyStyle.body.border.color = LV_COLOR_WHITE;

  for(size_t y = 0; y < 6; y++) {
    for(size_t x = 0; x < 6; x++) {
      lv_obj_t* tileObj = lv_obj_create(field, NULL);
      lv_obj_set_pos(tileObj, x * 40, y * 40);
      lv_obj_set_size(tileObj, 40, 40);
      lv_obj_set_style(tileObj, &greyStyle);
    }
  }

  lv_obj_t *robotLed = lv_led_create(field, NULL);
  lv_led_on(robotLed);
  lv_obj_set_size(robotLed, 16, 16);

  lv_style_copy(&ledStyle, &lv_style_plain);
  ledStyle.body.radius = LV_RADIUS_CIRCLE;
  ledStyle.body.main_color = LV_COLOR_BLUE;
  ledStyle.body.grad_color = LV_COLOR_BLUE;
  ledStyle.body.border.color = LV_COLOR_WHITE;
  ledStyle.body.border.width = 2;
  ledStyle.body.border.opa = LV_OPA_100;
  lv_obj_set_style(robotLed, &ledStyle);

  lv_obj_t *targetLed = lv_led_create(field, NULL);
  lv_led_on(targetLed);
  lv_obj_set_size(targetLed, 16, 16);

  lv_style_copy(&targetStyle, &ledStyle);
  targetStyle.body.main_color = LV_COLOR_RED;
  targetStyle.body.grad_color = LV_COLOR_RED;
  lv_obj_set_style(targetLed, &targetStyle);


  std::vector<lv_point_t> linePoints = {{0, 0}, {0, 0}};

  lv_obj_t *robotLine = lv_line_create(field, NULL);
  lv_line_set_points(robotLine, linePoints.data(), linePoints.size());
  lv_obj_set_pos(robotLine, 0, 0);

  lv_style_copy(&lineStyle, &lv_style_plain);
  lineStyle.line.width = 3;
  lineStyle.line.opa = LV_OPA_100;
  lineStyle.line.color = LV_COLOR_BLUE;
  lv_obj_set_style(robotLine, &lineStyle);


  lv_obj_t *statusLabel = lv_label_create(scr, NULL);
  lv_style_copy(&textStyle, &lv_style_plain);
  textStyle.text.color = LV_COLOR_WHITE;
  textStyle.text.opa = LV_OPA_100;
  lv_obj_set_style(statusLabel, &textStyle);
  lv_label_set_text(statusLabel, "No Data");
  lv_obj_align(statusLabel, scr, LV_ALIGN_IN_LEFT_MID, 0, 0);

  while(true){
    auto robotPos = robot::odometry->get();
    lv_obj_set_pos(robotLed, ((robotPos.x + 1) * 5.0/3.0) - (double)lv_obj_get_width(robotLed)/2, ((robotPos.y - 18) * -5.0/3.0) - (double)lv_obj_get_height(robotLed)/2 - 1);

    auto targetPos = robot::poseController->getTarget();
    lv_obj_set_pos(targetLed, ((targetPos.x + 1) * 5.0/3.0) - (double)lv_obj_get_width(targetLed)/2, ((targetPos.y - 18) * -5.0/3.0) - (double)lv_obj_get_height(targetLed)/2 - 1);

    linePoints[0] = {(int16_t)((robotPos.x + 1) * 5.0/3.0), (int16_t)(((robotPos.x + 1) * 5.0/3.0) - 1.5)};
    double dirY = 30 * cos(robotPos.theta);
    double dirX = 30 * sin(robotPos.theta);
    linePoints[1] = {(int16_t)(dirX + linePoints[0].x), (int16_t)(-dirY + linePoints[0].y)};
    lv_line_set_points(robotLine, linePoints.data(), linePoints.size());
    lv_obj_invalidate(robotLine);

    double dx = targetPos.x - robotPos.x;
    double dy = targetPos.y - robotPos.y;

    std::string text =
    "X_r: " + std::to_string(robotPos.x) + "\n" +
    "Y_r: " + std::to_string(robotPos.y) + "\n" +
    "Theta_r: " + std::to_string(robotPos.theta * 180/M_PI) + "\n" +
    "X_t: " + std::to_string(targetPos.x) + "\n" +
    "Y_t: " + std::to_string(targetPos.y) + "\n" +
    "Theta_t: " + std::to_string(targetPos.theta * 180/M_PI) + "\n" +
    "d: " + std::to_string(sqrt(dx * dx + dy * dy)) + "\n" +
    "phi: " + std::to_string(atan2(dy, dx) * 180/M_PI) + "\n" +
    "dTheta: " + std::to_string((targetPos.theta - robotPos.theta) * 180/M_PI) + "\n";

    lv_label_set_text(statusLabel, text.c_str());
    lv_obj_align(statusLabel, NULL, LV_ALIGN_IN_LEFT_MID, 0, 0);

    pros::delay(100);
  }
}
