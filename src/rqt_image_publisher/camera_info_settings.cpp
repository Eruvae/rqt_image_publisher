#include "rqt_image_publisher/camera_info_settings.h"
#include "ui_camera_info_settings.h"

#include <QInputDialog>
#include <ros/console.h>

namespace rqt_image_publisher
{

CameraInfoSettingsDialog::CameraInfoSettingsDialog(QMap<QString, QVariant> *presets, sensor_msgs::CameraInfo *camInfo, QWidget *parent) :
  presets(presets),
  camInfo(camInfo),
  QDialog(parent),
  ui(new Ui::CameraInfoSettingsDialogUi)
{
  ui->setupUi(this);
  D_SpinBoxes = {ui->D1, ui->D2, ui->D3, ui->D4, ui->D5, ui->D6, ui->D7, ui->D8};
  K_SpinBoxes = {ui->K1, ui->K2, ui->K3, ui->K4, ui->K5, ui->K6, ui->K7, ui->K8, ui->K9};
  R_SpinBoxes = {ui->R1, ui->R2, ui->R3, ui->R4, ui->R5, ui->R6, ui->R7, ui->R8, ui->R9};
  P_SpinBoxes = {ui->P1, ui->P2, ui->P3, ui->P4, ui->P5, ui->P6, ui->P7, ui->P8, ui->P9, ui->P10, ui->P11, ui->P12};
}

CameraInfoSettingsDialog::~CameraInfoSettingsDialog()
{
  delete ui;
}

void CameraInfoSettingsDialog::cameraInfoToUi()
{
  // load presets
  for (const QString &key : presets->keys())
  {
    ui->presetComboBox->addItem(key);
  }
  ui->distortionModelComboBox->setCurrentText(QString::fromStdString(camInfo->distortion_model));
  for (size_t i = 0; i < camInfo->D.size() && i < D_SpinBoxes.size(); i++)
    D_SpinBoxes[i]->setValue(camInfo->D[i]);
  for (size_t i = 0; i < camInfo->K.size() && i < K_SpinBoxes.size(); i++)
    K_SpinBoxes[i]->setValue(camInfo->K[i]);
  for (size_t i = 0; i < camInfo->R.size() && i < R_SpinBoxes.size(); i++)
    R_SpinBoxes[i]->setValue(camInfo->R[i]);
  for (size_t i = 0; i < camInfo->P.size() && i < P_SpinBoxes.size(); i++)
    P_SpinBoxes[i]->setValue(camInfo->P[i]);

  ui->binning_x->setValue(camInfo->binning_x);
  ui->binning_y->setValue(camInfo->binning_y);

  ui->roi_x_offset->setValue(camInfo->roi.x_offset);
  ui->roi_y_offset->setValue(camInfo->roi.y_offset);
  ui->roi_height->setValue(camInfo->roi.height);
  ui->roi_width->setValue(camInfo->roi.width);
  ui->roi_rectify->setChecked(camInfo->roi.do_rectify);

}

void CameraInfoSettingsDialog::uiToCameraInfo()
{
  camInfo->distortion_model = ui->distortionModelComboBox->currentText().toStdString();

  size_t D_count = D_SpinBoxes.size();
  if (camInfo->distortion_model == "plumb_bob")
    D_count = 5; // copy only first 5 parameters for plumb_bob

  camInfo->D.clear();
  camInfo->D.reserve(D_count);
  for (size_t i = 0; i < D_count; i++)
    camInfo->D.push_back(D_SpinBoxes[i]->value());

  for (size_t i = 0; i < camInfo->K.size() && i < K_SpinBoxes.size(); i++)
    camInfo->K[i] = K_SpinBoxes[i]->value();
  for (size_t i = 0; i < camInfo->R.size() && i < R_SpinBoxes.size(); i++)
    camInfo->R[i] = R_SpinBoxes[i]->value();
  for (size_t i = 0; i < camInfo->P.size() && i < P_SpinBoxes.size(); i++)
    camInfo->P[i] = P_SpinBoxes[i]->value();

  camInfo->binning_x = ui->binning_x->value();
  camInfo->binning_y = ui->binning_y->value();

  camInfo->roi.x_offset = ui->roi_x_offset->value();
  camInfo->roi.y_offset = ui->roi_y_offset->value();
  camInfo->roi.height = ui->roi_height->value();
  camInfo->roi.width = ui->roi_width->value();
  camInfo->roi.do_rectify = ui->roi_rectify->isChecked();
}

void CameraInfoSettingsDialog::on_confirmButtonBox_accepted()
{
  uiToCameraInfo();
  close();
}

void CameraInfoSettingsDialog::on_confirmButtonBox_rejected()
{
  close();
}

void CameraInfoSettingsDialog::on_savePresetButton_clicked()
{
  bool ok;
  QString text = QInputDialog::getText(this, "Save preset", "Preset name", QLineEdit::Normal, QString(), &ok);
  if (ok && !text.isEmpty())
    ROS_INFO_STREAM(text.toStdString());
}

}



