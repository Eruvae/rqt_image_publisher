#include "rqt_image_publisher/rqt_image_publisher.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QFileDialog>
#include <QImage>
#include <QThread>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace rqt_image_publisher
{

RqtImagePublisherWidget::RqtImagePublisherWidget(RqtImagePublisher* plugin, QWidget* parent, Qt::WindowFlags f)
  : QWidget(parent, f), plugin(plugin)
{
}

void RqtImagePublisherWidget::resizeEvent(QResizeEvent *event)
{
  if (plugin->image_qpix.isNull())
      return;

  int w = plugin->ui.imageView->width();
  int h = plugin->ui.imageView->height();
  plugin->ui.imageView->setPixmap(plugin->image_qpix.scaled(w,h,Qt::KeepAspectRatio));
}

RqtImagePublisher::RqtImagePublisher() :
  rqt_gui_cpp::Plugin(),
  widget(Q_NULLPTR),
  folder_model(Q_NULLPTR)
{
  setObjectName("RqtImagePublisher");
}

void RqtImagePublisher::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget = new RqtImagePublisherWidget(this);
  // extend the widget with all attributes and children from UI file
  ui.setupUi(widget);
  ui.settingsWidget->hide();
  // add widget to the user interface
  context.addWidget(widget);

  imt = new image_transport::ImageTransport(getNodeHandle());

  connect(ui.selectFolderButton, SIGNAL(clicked()), this, SLOT(on_selectFolderButton_clicked()));
  connect(ui.fileTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(on_fileTreeView_doubleClicked(QModelIndex)));
  connect(ui.previousImageButton, SIGNAL(clicked()), this, SLOT(on_previousImageButton_clicked()));
  connect(ui.nextImageButton, SIGNAL(clicked()), this, SLOT(on_nextImageButton_clicked()));
  connect(ui.publishButton, SIGNAL(clicked()), this, SLOT(on_publishButton_clicked()));
  connect(ui.openSettingsButton, SIGNAL(clicked()), this, SLOT(on_openSettingsButton_clicked()));
  connect(ui.settingsApplyButton, SIGNAL(clicked()), this, SLOT(on_settingsApplyButton_clicked()));
  connect(ui.settingsCancelButton, SIGNAL(clicked()), this, SLOT(on_settingsCancelButton_clicked()));
  connect(ui.publishContinouslyCheckBox, SIGNAL(toggled(bool)), this, SLOT(on_publishContinouslyCheckBox_toggled(bool)));
  connect(ui.rotateImagesCheckBox, SIGNAL(toggled(bool)), this, SLOT(on_rotateImagesCheckBox_toggled(bool)));
}

void RqtImagePublisher::shutdownPlugin()
{
  // unregister all publishers here
  image_pub.shutdown();
  delete imt;
}

void RqtImagePublisher::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("imageTopic", settings.imageTopic);
  instance_settings.setValue("frameId", settings.frameId);
  instance_settings.setValue("publishOnLoad", settings.publishOnLoad);
  instance_settings.setValue("publishLatched", settings.publishLatched);
  instance_settings.setValue("publishContinously", settings.publishContinously);
  instance_settings.setValue("publishingFrequency", settings.publishingFrequency);
  instance_settings.setValue("rotateImages", settings.rotateImages);
  instance_settings.setValue("rotateBackwards", settings.rotateBackwards);
  instance_settings.setValue("rotationFrequency", settings.rotationFrequency);
  instance_settings.setValue("scaleWidth", settings.scaleWidth);
  instance_settings.setValue("width", settings.width);
  instance_settings.setValue("scaleHeight", settings.scaleHeight);
  instance_settings.setValue("height", settings.height);
  instance_settings.setValue("keepRatio", settings.keepRatio);
}

void RqtImagePublisher::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  settings.imageTopic = instance_settings.value("imageTopic", "/image").toString();
  settings.frameId = instance_settings.value("frameId", "").toString();
  settings.publishOnLoad = instance_settings.value("publishOnLoad", false).toBool();
  settings.publishLatched = instance_settings.value("publishLatched", false).toBool();
  settings.publishContinously = instance_settings.value("publishContinously", false).toBool();
  settings.publishingFrequency = instance_settings.value("publishingFrequency", 1.0).toDouble();
  settings.rotateImages = instance_settings.value("rotateImages", false).toBool();
  settings.rotateBackwards = instance_settings.value("rotateBackwards", false).toBool();
  settings.rotationFrequency = instance_settings.value("rotationFrequency", 1.0).toDouble();
  settings.scaleWidth = instance_settings.value("scaleWidth", false).toBool();
  settings.width = instance_settings.value("width", 640).toInt();
  settings.scaleHeight = instance_settings.value("scaleHeight", false).toBool();
  settings.height = instance_settings.value("height", 480).toInt();
  settings.keepRatio = instance_settings.value("keepRatio", true).toBool();

  pluginSettingsToUi();
  applySettings();
}

bool RqtImagePublisher::hasConfiguration() const
{
  return true;
}

void RqtImagePublisher::triggerConfiguration()
{
  ui.settingsWidget->setHidden(!ui.settingsWidget->isHidden());
}

void RqtImagePublisher::on_selectFolderButton_clicked()
{
  QString dir_path = QFileDialog::getExistingDirectory(widget);
  if (dir_path.isEmpty())
    return;

  ui.imageView->clear();
  image_qimg = QImage();
  image_qpix = QPixmap();
  ui.previousImageButton->setEnabled(false);
  ui.nextImageButton->setEnabled(false);
  ui.publishButton->setEnabled(false);
  ui.curImageLabel->setText("0/0");

  QFileSystemModel *model = new QFileSystemModel;
  model->setRootPath(dir_path);
  ui.fileTreeView->setModel(model);
  ui.fileTreeView->setRootIndex(model->index(dir_path));
  ui.fileTreeView->hideColumn(1);
  ui.fileTreeView->hideColumn(2);
  ui.fileTreeView->hideColumn(3);

  if (folder_model)
    delete folder_model;

  folder_model = model;
}

void RqtImagePublisher::on_fileTreeView_doubleClicked(const QModelIndex &index)
{
  loadImage(index);
}

void RqtImagePublisher::on_previousImageButton_clicked()
{
  if (!selected_image.isValid())
    return;

  int num_imgs_in_folder = folder_model->rowCount(selected_image.parent());
  int item_row = selected_image.row();

  auto previous_row = [&](int row) { return row > 0 ? row - 1 : num_imgs_in_folder - 1; };

  for (int cur_row = previous_row(item_row); cur_row != item_row; cur_row = previous_row(cur_row))
  {
    QModelIndex index = folder_model->index(cur_row, selected_image.column(), selected_image.parent());
    if (loadImage(index))
      break;
  }
}

void RqtImagePublisher::on_nextImageButton_clicked()
{
  if (!selected_image.isValid())
    return;

  int num_imgs_in_folder = folder_model->rowCount(selected_image.parent());
  int item_row = selected_image.row();

  auto next_row = [&](int row) { return (row + 1) % num_imgs_in_folder; };

  for (int cur_row = next_row(item_row); cur_row != item_row; cur_row = next_row(cur_row))
  {
    QModelIndex index = folder_model->index(cur_row, selected_image.column(), selected_image.parent());
    if (loadImage(index))
      break;
  }
}

void RqtImagePublisher::on_publishButton_clicked()
{
  image_ros.header.stamp = ros::Time::now();
  image_pub.publish(image_ros);
}

void RqtImagePublisher::on_openSettingsButton_clicked()
{
  ui.settingsWidget->setHidden(!ui.settingsWidget->isHidden());
}

void RqtImagePublisher::on_settingsCancelButton_clicked()
{
  pluginSettingsToUi();
  ui.settingsWidget->hide();
}

void RqtImagePublisher::on_settingsApplyButton_clicked()
{
  uiToPluginSettings();
  ui.settingsWidget->hide();
  applySettings();
}

void RqtImagePublisher::on_publishContinouslyCheckBox_toggled(bool checked)
{
  ui.publishLatchedCheckBox->setEnabled(!checked && !ui.rotateImagesCheckBox->isChecked());
  ui.publishingFrequencySpinBox->setEnabled(checked && !ui.rotateImagesCheckBox->isChecked());
}

void RqtImagePublisher::on_rotateImagesCheckBox_toggled(bool checked)
{
  ui.publishLatchedCheckBox->setEnabled(!checked && !ui.publishContinouslyCheckBox->isChecked());
  ui.publishContinouslyCheckBox->setEnabled(!checked);
  ui.publishingFrequencySpinBox->setEnabled(!checked && ui.publishContinouslyCheckBox->isChecked());

  ui.rotateBackwardsCheckBox->setEnabled(checked);
  ui.rotationFrequencySpinBox->setEnabled(checked);
}


bool RqtImagePublisher::loadImage(const QModelIndex &index)
{
  QString path = folder_model->filePath(index);
  QImage new_image;
  if (!new_image.load(path))
  {
    ROS_INFO_STREAM("Selected file is not an image");
    return false;
  }

  image_qimg = new_image;

  // store loaded image as pixmap for display
  image_qpix = QPixmap::fromImage(image_qimg);
  int w = ui.imageView->width();
  int h = ui.imageView->height();
  ui.imageView->setPixmap(image_qpix.scaled(w,h,Qt::KeepAspectRatio));

  generateRosImage(); // convert image to ROS message for publishing

  if (settings.publishOnLoad)
    image_pub.publish(image_ros);

  selected_image = index;
  int num_imgs_in_folder = folder_model->rowCount(selected_image.parent());
  int item_row = selected_image.row();
  ui.curImageLabel->setText(QString::asprintf("%d/%d", item_row + 1, num_imgs_in_folder));
  ui.fileTreeView->setCurrentIndex(selected_image);

  ui.previousImageButton->setEnabled(true);
  ui.nextImageButton->setEnabled(true);
  ui.publishButton->setEnabled(true);

  return true;
}

bool RqtImagePublisher::generateRosImage()
{
  if (image_qimg.isNull())
    return false;

  QImage qt_image_rgb8 = image_qimg.convertToFormat(QImage::Format_RGB888);
  if (settings.scaleWidth && settings.scaleHeight)
  {
    Qt::AspectRatioMode aspectMode = settings.keepRatio ? Qt::AspectRatioMode::KeepAspectRatio : Qt::AspectRatioMode::IgnoreAspectRatio;
    qt_image_rgb8 = qt_image_rgb8.scaled(settings.width, settings.height, aspectMode);
  }
  else if (settings.scaleWidth)
  {
    qt_image_rgb8 = qt_image_rgb8.scaledToWidth(settings.width);
  }
  else if (settings.scaleHeight)
  {
    qt_image_rgb8 = qt_image_rgb8.scaledToHeight(settings.height);
  }
  image_ros.height = qt_image_rgb8.height();
  image_ros.width = qt_image_rgb8.width();
  image_ros.encoding = sensor_msgs::image_encodings::RGB8;
  image_ros.step = qt_image_rgb8.bytesPerLine();
  image_ros.data.resize(qt_image_rgb8.byteCount());
  memcpy(image_ros.data.data(), qt_image_rgb8.bits(), qt_image_rgb8.byteCount());
  image_ros.header.stamp = ros::Time::now();
  image_ros.header.frame_id = settings.frameId.toStdString();
  return true;
}

void RqtImagePublisher::pluginSettingsToUi()
{
  ui.imageTopicTextEdit->setText(settings.imageTopic);
  ui.frameIdTextEdit->setText(settings.frameId);
  ui.publishOnLoadCheckBox->setChecked(settings.publishOnLoad);
  ui.publishLatchedCheckBox->setChecked(settings.publishLatched);
  ui.publishContinouslyCheckBox->setChecked(settings.publishContinously);
  ui.publishingFrequencySpinBox->setValue(settings.publishingFrequency);
  ui.rotateImagesCheckBox->setChecked(settings.rotateImages);
  ui.rotateBackwardsCheckBox->setChecked(settings.rotateBackwards);
  ui.rotationFrequencySpinBox->setValue(settings.rotationFrequency);
  ui.scaleWidthCheckBox->setChecked(settings.scaleWidth);
  ui.widthSpinBox->setValue(settings.width);
  ui.scaleHeightCheckBox->setChecked(settings.scaleHeight);
  ui.heightSpinBox->setValue(settings.height);
  ui.keepRatioCheckBox->setChecked(settings.keepRatio);
}

void RqtImagePublisher::uiToPluginSettings()
{
  settings.imageTopic = ui.imageTopicTextEdit->text();
  settings.frameId = ui.frameIdTextEdit->text();
  settings.publishOnLoad = ui.publishOnLoadCheckBox->isChecked();
  settings.publishLatched = ui.publishLatchedCheckBox->isChecked();
  settings.publishContinously = ui.publishContinouslyCheckBox->isChecked();
  settings.publishingFrequency = ui.publishingFrequencySpinBox->value();
  settings.rotateImages = ui.rotateImagesCheckBox->isChecked();
  settings.rotateBackwards = ui.rotateBackwardsCheckBox->isChecked();
  settings.rotationFrequency = ui.rotationFrequencySpinBox->value();
  settings.scaleWidth = ui.scaleWidthCheckBox->isChecked();
  settings.width = ui.widthSpinBox->value();
  settings.scaleHeight = ui.scaleHeightCheckBox->isChecked();
  settings.height = ui.heightSpinBox->value();
  settings.keepRatio = ui.keepRatioCheckBox->isChecked();
}

void RqtImagePublisher::applySettings()
{
  image_pub.shutdown();
  bool latched = settings.publishLatched && !settings.publishContinously && !settings.rotateImages;
  image_pub = imt->advertise(settings.imageTopic.toStdString(), 1, latched);
  generateRosImage();
}

} // namespace rqt_image_publisher

PLUGINLIB_EXPORT_CLASS(rqt_image_publisher::RqtImagePublisher, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(rqt_image_publisher, RqtImagePublisher, rqt_image_publisher::RqtImagePublisher, rqt_gui_cpp::Plugin)
