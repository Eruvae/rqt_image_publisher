#ifndef CAMERA_INFO_SETTINGS_H
#define CAMERA_INFO_SETTINGS_H

#include <QDialog>
#include <sensor_msgs/CameraInfo.h>
#include "ui_camera_info_settings.h"

namespace Ui {
class CameraInfoSettingsDialogUi;
}

namespace rqt_image_publisher
{

class CameraInfoSettingsDialog : public QDialog
{
  Q_OBJECT
public:
  explicit CameraInfoSettingsDialog(QMap<QString, QVariant> *presets, sensor_msgs::CameraInfo *camInfo, QWidget *parent = nullptr);
  ~CameraInfoSettingsDialog();

  void cameraInfoToUi();
  void uiToCameraInfo();

private slots:
  void on_confirmButtonBox_accepted();
  void on_confirmButtonBox_rejected();

  void on_savePresetButton_clicked();

private:
  Ui::CameraInfoSettingsDialogUi *ui;
  QMap<QString, QVariant> *presets;
  sensor_msgs::CameraInfo *camInfo;

  QVector<QDoubleSpinBox*> D_SpinBoxes;
  QVector<QDoubleSpinBox*> K_SpinBoxes;
  QVector<QDoubleSpinBox*> R_SpinBoxes;
  QVector<QDoubleSpinBox*> P_SpinBoxes;
};

}

#endif // CAMERA_INFO_SETTINGS_H
