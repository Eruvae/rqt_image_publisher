#include "rqt_image_publisher/rqt_image_publisher.h"

#include <pluginlib/class_list_macros.h>
#include <QFileDialog>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace rqt_image_publisher
{

RqtImagePublisherWidget::RqtImagePublisherWidget(RqtImagePublisher* plugin, QWidget* parent, Qt::WindowFlags f)
  : QWidget(parent, f), plugin(plugin)
{
}

void RqtImagePublisherWidget::resizeEvent(QResizeEvent *event)
{
  plugin->rescaleImageLabel();
}

RqtImagePublisher::RqtImagePublisher() :
  rqt_gui_cpp::Plugin(),
  widget(Q_NULLPTR),
  folder_model(Q_NULLPTR),
  publishingTimer(this),
  image_loaded(false)
{
  setObjectName("RqtImagePublisher");
}

void RqtImagePublisher::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  // QStringList argv = context.argv();
  // create QWidget
  widget = new RqtImagePublisherWidget(this);
  // extend the widget with all attributes and children from UI file
  ui.setupUi(widget);
  ui.settingsWidget->hide();
  ui.filterListWidget->hide();
  // add widget to the user interface
  context.addWidget(widget);

  imt = new image_transport::ImageTransport(getNodeHandle());

  connect(ui.selectFolderButton, SIGNAL(clicked()), this, SLOT(on_selectFolderButton_clicked()));
  connect(ui.fileTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(on_fileTreeView_doubleClicked(QModelIndex)));
  connect(ui.previousImageButton, SIGNAL(clicked()), this, SLOT(on_previousImageButton_clicked()));
  connect(ui.nextImageButton, SIGNAL(clicked()), this, SLOT(on_nextImageButton_clicked()));
  connect(ui.publishButton, SIGNAL(clicked()), this, SLOT(on_publishButton_clicked()));
  connect(ui.openSettingsButton, SIGNAL(clicked()), this, SLOT(on_openSettingsButton_clicked()));
  connect(ui.filterListButton, SIGNAL(clicked()), this, SLOT(on_filterListButton_clicked()));
  connect(ui.settingsApplyButton, SIGNAL(clicked()), this, SLOT(on_settingsApplyButton_clicked()));
  connect(ui.settingsCancelButton, SIGNAL(clicked()), this, SLOT(on_settingsCancelButton_clicked()));
  connect(ui.filterListApplyButton, SIGNAL(clicked()), this, SLOT(on_filterListApplyButton_clicked()));
  connect(ui.filterListCancelButton, SIGNAL(clicked()), this, SLOT(on_filterListCancelButton_clicked()));
  connect(ui.publishContinuouslyCheckBox, SIGNAL(toggled(bool)), this, SLOT(on_publishContinuouslyCheckBox_toggled(bool)));
  connect(ui.rotateImagesCheckBox, SIGNAL(toggled(bool)), this, SLOT(on_rotateImagesCheckBox_toggled(bool)));
  connect(&publishingTimer, SIGNAL(timeout()), this, SLOT(on_publishingTimer_timeout()));
}

void RqtImagePublisher::shutdownPlugin()
{
  // unregister all publishers here
  image_pub.shutdown();
  delete imt;
  if (folder_model)
    delete folder_model;
}

void RqtImagePublisher::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("imageTopic", settings.imageTopic);
  instance_settings.setValue("frameId", settings.frameId);
  instance_settings.setValue("imageType", settings.imageType);
  instance_settings.setValue("colorEnc", settings.colorEnc);
  instance_settings.setValue("colorAlphaEnc", settings.colorAlphaEnc);
  instance_settings.setValue("monoEnc", settings.monoEnc);
  instance_settings.setValue("depthEnc", settings.depthEnc);
  instance_settings.setValue("publishOnLoad", settings.publishOnLoad);
  instance_settings.setValue("publishLatched", settings.publishLatched);
  instance_settings.setValue("publishContinuously", settings.publishContinuously);
  instance_settings.setValue("publishingFrequency", settings.publishingFrequency);
  instance_settings.setValue("rotateImages", settings.rotateImages);
  instance_settings.setValue("rotateBackwards", settings.rotateBackwards);
  instance_settings.setValue("startSlideshowOnLoad", settings.startSlideshowOnLoad);
  instance_settings.setValue("rotationFrequency", settings.rotationFrequency);
  instance_settings.setValue("scaleWidth", settings.scaleWidth);
  instance_settings.setValue("width", settings.width);
  instance_settings.setValue("scaleHeight", settings.scaleHeight);
  instance_settings.setValue("height", settings.height);
  instance_settings.setValue("keepRatio", settings.keepRatio);
  instance_settings.setValue("filters", settings.filters);
  instance_settings.setValue("lastFolder", settings.lastFolder);
}

void RqtImagePublisher::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  settings.imageTopic = instance_settings.value("imageTopic", "/image").toString();
  settings.frameId = instance_settings.value("frameId", "").toString();
  settings.imageType = instance_settings.value("imageType", 0).toInt();
  settings.colorEnc = instance_settings.value("colorEnc", 0).toInt();
  settings.colorAlphaEnc = instance_settings.value("colorAlphaEnc", 0).toInt();
  settings.monoEnc = instance_settings.value("monoEnc", 0).toInt();
  settings.depthEnc = instance_settings.value("depthEnc", 0).toInt();
  settings.publishOnLoad = instance_settings.value("publishOnLoad", false).toBool();
  settings.publishLatched = instance_settings.value("publishLatched", false).toBool();
  settings.publishContinuously = instance_settings.value("publishContinuously", false).toBool();
  settings.publishingFrequency = instance_settings.value("publishingFrequency", 1.0).toDouble();
  settings.rotateImages = instance_settings.value("rotateImages", false).toBool();
  settings.rotateBackwards = instance_settings.value("rotateBackwards", false).toBool();
  settings.startSlideshowOnLoad = instance_settings.value("startSlideshowOnLoad", false).toBool();
  settings.rotationFrequency = instance_settings.value("rotationFrequency", 1.0).toDouble();
  settings.scaleWidth = instance_settings.value("scaleWidth", false).toBool();
  settings.width = instance_settings.value("width", 640).toInt();
  settings.scaleHeight = instance_settings.value("scaleHeight", false).toBool();
  settings.height = instance_settings.value("height", 480).toInt();
  settings.keepRatio = instance_settings.value("keepRatio", true).toBool();
  settings.filters = instance_settings.value("filters", QStringList()).toStringList();
  settings.lastFolder = instance_settings.value("lastFolder", QString()).toString();

  ui.filterListTextEdit->setPlainText(settings.filters.join('\n'));
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
  QString dir_path = QFileDialog::getExistingDirectory(widget, QString(), settings.lastFolder);
  if (dir_path.isEmpty())
    return;

  settings.lastFolder = dir_path;

  clearSelectedImage();

  QFileSystemModel *model = new QFileSystemModel;
  model->setRootPath(dir_path);
  model->setNameFilterDisables(false);
  model->setNameFilters(settings.filters);
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
  if (settings.rotateImages) // this is start / stop slideshow
  {
    if (publishingTimer.isActive()) // stop slideshow
    {
      stopSlideshow();
    }
    else // start slideshow
    {
      startSlideshow();
    }
  }
  else if (settings.publishContinuously)
  {
    if (publishingTimer.isActive()) // stop publishing
    {
      stopPublishing();
    }
    else // start publishing
    {
      startPublishing();
    }
  }
  else
  {
    image_ros.header.stamp = ros::Time::now();
    image_pub.publish(image_ros);
  }
}

void RqtImagePublisher::on_openSettingsButton_clicked()
{
  ui.settingsWidget->setHidden(!ui.settingsWidget->isHidden());
  rescaleImageLabel();
}

void RqtImagePublisher::on_filterListButton_clicked()
{
  ui.filterListWidget->setHidden(!ui.filterListWidget->isHidden());
  rescaleImageLabel();
}

void RqtImagePublisher::on_settingsCancelButton_clicked()
{
  pluginSettingsToUi();
  ui.settingsWidget->hide();
  rescaleImageLabel();
}

void RqtImagePublisher::on_settingsApplyButton_clicked()
{
  uiToPluginSettings();
  ui.settingsWidget->hide();
  applySettings();
}

void RqtImagePublisher::on_filterListCancelButton_clicked()
{
  ui.filterListTextEdit->setPlainText(settings.filters.join('\n'));
  ui.filterListWidget->hide();
  rescaleImageLabel();
}

void RqtImagePublisher::on_filterListApplyButton_clicked()
{
  //ROS_INFO_STREAM("Selected image before: " << selected_image.isValid() << ", " << selected_image.row());
  //QString selected_path = folder_model->filePath(selected_image);
  QString content = ui.filterListTextEdit->toPlainText();
  settings.filters = content.split('\n', QString::SkipEmptyParts);
  ui.filterListTextEdit->setPlainText(settings.filters.join('\n'));

  if (folder_model)
  {
    folder_model->setNameFilters(settings.filters);
    //QModelIndex index = folder_model->index(selected_path);
    //ROS_INFO_STREAM("Selected image after: " << index.isValid() << ", " << index.row());
    clearSelectedImage();
  }
  ui.filterListWidget->hide();
  rescaleImageLabel();
}

void RqtImagePublisher::on_publishContinuouslyCheckBox_toggled(bool checked)
{
  ui.publishLatchedCheckBox->setEnabled(!checked && !ui.rotateImagesCheckBox->isChecked());
  ui.publishingFrequencySpinBox->setEnabled(checked && !ui.rotateImagesCheckBox->isChecked());
}

void RqtImagePublisher::on_rotateImagesCheckBox_toggled(bool checked)
{
  ui.publishOnLoadCheckBox->setEnabled(!checked);
  ui.publishLatchedCheckBox->setEnabled(!checked && !ui.publishContinuouslyCheckBox->isChecked());
  ui.publishContinuouslyCheckBox->setEnabled(!checked);
  ui.publishingFrequencySpinBox->setEnabled(!checked && ui.publishContinuouslyCheckBox->isChecked());

  ui.rotateBackwardsCheckBox->setEnabled(checked);
  ui.startSlideshowOnLoadCheckBox->setEnabled(checked);
  ui.rotationFrequencySpinBox->setEnabled(checked);
}

void RqtImagePublisher::on_publishingTimer_timeout()
{
  if (settings.rotateImages)
  {
    if (settings.rotateBackwards)
      on_previousImageButton_clicked();
    else
      on_nextImageButton_clicked();

    image_pub.publish(image_ros);
  }
  else if (settings.publishContinuously)
  {
    image_ros.header.stamp = ros::Time::now();
    image_pub.publish(image_ros);
  }
}

bool RqtImagePublisher::loadImage(const QModelIndex &index)
{
  if (folder_model->isDir(index))
  {
    ROS_INFO_STREAM("Selection is directory");
    return false;
  }

  QString path = folder_model->filePath(index);


  cv::Mat new_image = cv::imread(path.toStdString(), cv::IMREAD_UNCHANGED);   // Read the file
  if (!new_image.data)
  {
    ROS_INFO_STREAM("OpenCV could not read file");
    return false;
  }

  image_cv = new_image;
  ROS_INFO_STREAM("Depth: " << image_cv.depth() << ", channels: " << image_cv.channels());

  /*QImage new_image;
  if (!new_image.load(path))
  {
    ROS_INFO_STREAM("Selected file is not an image");
    return false;
  }

  image_qimg = new_image;
  ROS_INFO_STREAM("Image format: " << image_qimg.format());*/

  generateCvBridgeImage();

  generatePixmap();

  // generateRosImage(); // convert image to ROS message for publishing

  setSelectedImage(index);

  ui.previousImageButton->setEnabled(true);
  ui.nextImageButton->setEnabled(true);
  ui.publishButton->setEnabled(true);

  if (settings.rotateImages)
  {
    if (settings.startSlideshowOnLoad && !publishingTimer.isActive())
    {
      startSlideshow();
    }
  }
  else if (settings.publishOnLoad)
  {
    if (!settings.publishContinuously)
    {
      image_pub.publish(image_ros);
    }
    else if (!publishingTimer.isActive())
    {
      startPublishing();
    }
  }

  image_loaded = true;
  return true;
}

cv::Mat RqtImagePublisher::convertToDepth(const cv::Mat &mat, int depth)
{
  cv::Mat mat_conv;
  int d = mat.depth();
  if (d == depth)
    mat_conv = mat;
  else if ((d == CV_8U || d == CV_8S) && (depth == CV_16U || depth == CV_16S))
    mat.convertTo(mat_conv, depth, 256.0);
  else if ((d == CV_16U || d == CV_16S) && (depth == CV_8U || depth == CV_8S))
    mat.convertTo(mat_conv, depth, 1.0/256.0);
  else if ((d == CV_32F || d == CV_64F) && (depth == CV_16U || depth == CV_16S))
    mat.convertTo(mat_conv, depth, 1000.0);
  else if ((d == CV_16U || d == CV_16S) && (depth == CV_32F || depth == CV_64F))
    mat.convertTo(mat_conv, depth, 1.0/1000.0);
  else if ((d == CV_32F || d == CV_64F) && (depth == CV_8U || depth == CV_8S))
    mat.convertTo(mat_conv, depth, 1000.0/256.0);
  else if ((d == CV_8U || d == CV_8S) && (depth == CV_32F || depth == CV_64F))
    mat.convertTo(mat_conv, depth, 256.0/1000.0);
  else
    mat.convertTo(mat_conv, depth);

  return mat_conv;
}

cv::Mat RqtImagePublisher::convertToEncoding(const cv::Mat &mat, Encoding enc, Encoding sourceEnc)
{
  cv::Mat mat_conv;
  int c = mat.channels();
  if (c == 1) // image is mono or depth
  {
    switch (enc)
    {
    case ENC_RGB8: case ENC_RGB16: cv::cvtColor(mat, mat_conv, CV_GRAY2RGB); break;
    case ENC_BGR8: case ENC_BGR16: cv::cvtColor(mat, mat_conv, CV_GRAY2BGR); break;
    case ENC_RGBA8: case ENC_RGBA16: cv::cvtColor(mat, mat_conv, CV_GRAY2RGBA); break;
    case ENC_BGRA8: case ENC_BGRA16: cv::cvtColor(mat, mat_conv, CV_GRAY2BGRA); break;
    default: mat_conv = mat;
    }
  }
  else if (c == 3) // image is bgr or rgb
  {
    if (sourceEnc == ENC_RGB8 || sourceEnc == ENC_RGB16)
    {
      switch (enc)
      {
      case ENC_BGR8: case ENC_BGR16: cv::cvtColor(mat, mat_conv, CV_RGB2BGR); break;
      case ENC_RGBA8: case ENC_RGBA16: cv::cvtColor(mat, mat_conv, CV_RGB2RGBA); break;
      case ENC_BGRA8: case ENC_BGRA16: cv::cvtColor(mat, mat_conv, CV_RGB2BGRA); break;
      case ENC_MONO8: case ENC_MONO16:
      case ENC_16UC1: case ENC_32FC1: cv::cvtColor(mat, mat_conv, CV_RGB2GRAY); break;
      default: mat_conv = mat;
      }
    }
    else
    {
      switch (enc)
      {
      case ENC_RGB8: case ENC_RGB16: cv::cvtColor(mat, mat_conv, CV_BGR2RGB); break;
      case ENC_RGBA8: case ENC_RGBA16: cv::cvtColor(mat, mat_conv, CV_BGR2RGBA); break;
      case ENC_BGRA8: case ENC_BGRA16: cv::cvtColor(mat, mat_conv, CV_BGR2BGRA); break;
      case ENC_MONO8: case ENC_MONO16:
      case ENC_16UC1: case ENC_32FC1: cv::cvtColor(mat, mat_conv, CV_BGR2GRAY); break;
      default: mat_conv = mat;
      }
    }
  }
  else if (c == 4) // image is bgra or rgba
  {
    if (sourceEnc == ENC_RGBA8 || sourceEnc == ENC_RGB16)
    {
      switch (enc)
      {
      case ENC_RGB8: case ENC_RGB16: cv::cvtColor(mat, mat_conv, CV_RGBA2RGB); break;
      case ENC_BGR8: case ENC_BGR16: cv::cvtColor(mat, mat_conv, CV_RGBA2BGR); break;
      case ENC_BGRA8: case ENC_BGRA16: cv::cvtColor(mat, mat_conv, CV_RGBA2BGRA); break;
      case ENC_MONO8: case ENC_MONO16:
      case ENC_16UC1: case ENC_32FC1: cv::cvtColor(mat, mat_conv, CV_RGBA2GRAY); break;
      default: mat_conv = mat;
      }
    }
    else
    {
      switch (enc)
      {
      case ENC_RGB8: case ENC_RGB16: cv::cvtColor(mat, mat_conv, CV_BGRA2RGB); break;
      case ENC_BGR8: case ENC_BGR16: cv::cvtColor(mat, mat_conv, CV_BGRA2BGR); break;
      case ENC_RGBA8: case ENC_RGBA16: cv::cvtColor(mat, mat_conv, CV_BGRA2RGBA); break;
      case ENC_MONO8: case ENC_MONO16:
      case ENC_16UC1: case ENC_32FC1: cv::cvtColor(mat, mat_conv, CV_BGRA2GRAY); break;
      default: mat_conv = mat;
      }
    }

  }
  else
  {
    ROS_ERROR("Only 1, 3 or 4 channel images supported");
    mat_conv = mat;
  }
  return mat_conv;
}

void RqtImagePublisher::resizeCvBridgeImage()
{
  ROS_INFO_STREAM(image_cvb.image.size());
  int curWidth = image_cvb.image.cols, curHeight = image_cvb.image.rows;
  if (curWidth == 0 || curHeight == 0) return;
  double aspectRatio = (double)curWidth / (double)curHeight;
  int targetWidth = settings.width, targetHeight = settings.height;
  if (settings.scaleWidth && settings.scaleHeight)
  {
    if (targetWidth == 0 || targetHeight == 0) return;
    if (settings.keepRatio)
    {
      double targetAspect = (double)targetWidth / (double)targetHeight;
      if (targetAspect > aspectRatio)
        targetWidth = (int)std::round(targetHeight * aspectRatio);
      else if (targetAspect < aspectRatio)
        targetHeight = (int)std::round(targetWidth / aspectRatio);
    }
    cv::resize(image_cvb.image, image_cvb.image, cv::Size(targetWidth, targetHeight));
  }
  else if (settings.scaleWidth)
  {
    if (targetWidth == 0) return;
    targetHeight = (int)std::round(targetWidth / aspectRatio);
    cv::resize(image_cvb.image, image_cvb.image, cv::Size(targetWidth, targetHeight));
  }
  else if (settings.scaleHeight)
  {
    if (targetHeight == 0) return;
    targetWidth = (int)std::round(targetHeight * aspectRatio);
    cv::resize(image_cvb.image, image_cvb.image, cv::Size(targetWidth, targetHeight));
  }
}

ImageType RqtImagePublisher::detectType(const cv::Mat &mat)
{
  int d = mat.depth();
  int c = mat.channels();
  if (c == 1)
  {
    if (d > CV_8S) // single channel images with bit depth > 8 are assumed to be depth images
      return IT_DEPTH;
    else
      return IT_MONO;
  }
  else if (c == 3)
    return IT_COLOR;
  else if (c == 4)
    return IT_COLOR_ALPHA;

  return IT_INVALID;
}

Encoding RqtImagePublisher::detectEncoding(const cv::Mat &mat, ImageType type)
{
  int d = mat.depth();
  switch(type)
  {
  case IT_COLOR:
    if (d > CV_8S) return ENC_BGR16;
    else return ENC_BGR8;
  case IT_COLOR_ALPHA:
    if (d > CV_8S) return ENC_BGRA16;
    else return ENC_BGRA8;
  case IT_MONO:
    if (d > CV_8S) return ENC_MONO16;
    else return ENC_MONO8;
  case IT_DEPTH:
    if (d == CV_32F || d == CV_64F) return ENC_32FC1;
    else return ENC_16UC1;
  default:
    return ENC_INVALID;
  }
}

bool RqtImagePublisher::generateCvBridgeImage()
{
  // determine type and encoding
  ImageType type = (settings.imageType == IT_AUTO) ? detectType(image_cv) : (ImageType)settings.imageType;
  Encoding enc;
  switch (type)
  {
  case IT_COLOR: enc = (Encoding)settings.colorEnc; break;
  case IT_COLOR_ALPHA: enc = (Encoding)settings.colorAlphaEnc; break;
  case IT_MONO: enc = (Encoding)settings.monoEnc; break;
  case IT_DEPTH: enc = (Encoding)settings.depthEnc; break;
  default: enc = ENC_INVALID;
  }
  if (enc == ENC_AUTO) enc = detectEncoding(image_cv, type);

  // set header and encoding
  image_cvb.header.frame_id = settings.frameId.toStdString();
  image_cvb.encoding = encodingToString(enc);

  // transform cv mat to depth
  switch (sensor_msgs::image_encodings::bitDepth(image_cvb.encoding))
  {
  case 8: image_cvb.image = convertToDepth(image_cv, CV_8U); break;
  case 16: image_cvb.image = convertToDepth(image_cv, CV_16U); break;
  case 32: image_cvb.image = convertToDepth(image_cv, CV_32F); break;
  case 64: image_cvb.image = convertToDepth(image_cv, CV_64F); break;
  default: image_cvb.image = convertToDepth(image_cv, CV_8U);
  }

  image_cvb.image = convertToEncoding(image_cvb.image, enc);

  resizeCvBridgeImage();

  image_cvb.toImageMsg(image_ros);

  ui.statusLineEdit->setText(QString::fromStdString("Image encoding: " + image_cvb.encoding));

  return true;
}

void RqtImagePublisher::generatePixmap()
{
  // convert to image with 8-bit unsigned depth
  cv::Mat mat_8u = convertToDepth(image_cvb.image, CV_8U);

  // convert to RGB
  mat_8u = convertToEncoding(mat_8u, ENC_RGB8, stringToEncoding(image_cvb.encoding));
  image_qimg = QImage(mat_8u.data, mat_8u.cols, mat_8u.rows, mat_8u.step, QImage::Format_RGB888);

  ROS_INFO_STREAM("Image size: " << mat_8u.cols << ", " << mat_8u.rows);

  // store loaded image as pixmap for display
  image_qpix = QPixmap::fromImage(image_qimg);
  rescaleImageLabel();
}

/*bool RqtImagePublisher::generateRosImage()
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
}*/

void RqtImagePublisher::setSelectedImage(QModelIndex index)
{
  selected_image = index;
  int num_imgs_in_folder = folder_model->rowCount(selected_image.parent());
  int item_row = selected_image.row();
  ui.curImageLabel->setText(QString::asprintf("%d/%d", item_row + 1, num_imgs_in_folder));
  ui.fileTreeView->setCurrentIndex(selected_image);
}

void RqtImagePublisher::clearSelectedImage()
{
  selected_image = QModelIndex();
  image_loaded = false;
  ui.imageView->clear();
  image_cv = cv::Mat();
  image_cvb.image = cv::Mat();
  image_qimg = QImage();
  image_qpix = QPixmap();
  ui.previousImageButton->setEnabled(false);
  ui.nextImageButton->setEnabled(false);
  ui.publishButton->setEnabled(false);
  ui.curImageLabel->setText("0/0");
  ui.fileTreeView->clearSelection();
}

void RqtImagePublisher::startSlideshow()
{
  image_ros.header.stamp = ros::Time::now();
  image_pub.publish(image_ros);
  publishingTimer.start();
  ui.publishButton->setText("Stop slideshow");
}

void RqtImagePublisher::stopSlideshow()
{
  publishingTimer.stop();
  ui.publishButton->setText("Start slideshow");
}

void RqtImagePublisher::startPublishing()
{
  image_ros.header.stamp = ros::Time::now();
  image_pub.publish(image_ros);
  publishingTimer.start();
  ui.publishButton->setText("Stop publishing");
}

void RqtImagePublisher::stopPublishing()
{
  publishingTimer.stop();
  ui.publishButton->setText("Start publishing");
}

void RqtImagePublisher::pluginSettingsToUi()
{
  ui.imageTopicTextEdit->setText(settings.imageTopic);
  ui.frameIdTextEdit->setText(settings.frameId);
  ui.typeComboBox->setCurrentIndex(settings.imageType);
  ui.colorEncComboBox->setCurrentIndex(encodingToIndex(IT_COLOR, settings.colorEnc));
  ui.colorAlphaEncComboBox->setCurrentIndex(encodingToIndex(IT_COLOR_ALPHA, settings.colorAlphaEnc));
  ui.monoEncComboBox->setCurrentIndex(encodingToIndex(IT_MONO, settings.monoEnc));
  ui.depthEncComboBox->setCurrentIndex(encodingToIndex(IT_DEPTH, settings.depthEnc));
  ui.publishOnLoadCheckBox->setChecked(settings.publishOnLoad);
  ui.publishLatchedCheckBox->setChecked(settings.publishLatched);
  ui.publishContinuouslyCheckBox->setChecked(settings.publishContinuously);
  ui.publishingFrequencySpinBox->setValue(settings.publishingFrequency);
  ui.rotateImagesCheckBox->setChecked(settings.rotateImages);
  ui.rotateBackwardsCheckBox->setChecked(settings.rotateBackwards);
  ui.startSlideshowOnLoadCheckBox->setChecked(settings.startSlideshowOnLoad);
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
  settings.imageType = ui.typeComboBox->currentIndex();
  settings.colorEnc = indexToEncoding(IT_COLOR, ui.colorEncComboBox->currentIndex());
  settings.colorAlphaEnc = indexToEncoding(IT_COLOR_ALPHA, ui.colorAlphaEncComboBox->currentIndex());
  settings.monoEnc = indexToEncoding(IT_MONO, ui.monoEncComboBox->currentIndex());
  settings.depthEnc = indexToEncoding(IT_DEPTH, ui.depthEncComboBox->currentIndex());
  settings.publishOnLoad = ui.publishOnLoadCheckBox->isChecked();
  settings.publishLatched = ui.publishLatchedCheckBox->isChecked();
  settings.publishContinuously = ui.publishContinuouslyCheckBox->isChecked();
  settings.publishingFrequency = ui.publishingFrequencySpinBox->value();
  settings.rotateImages = ui.rotateImagesCheckBox->isChecked();
  settings.rotateBackwards = ui.rotateBackwardsCheckBox->isChecked();
  settings.startSlideshowOnLoad = ui.startSlideshowOnLoadCheckBox->isChecked();
  settings.rotationFrequency = ui.rotationFrequencySpinBox->value();
  settings.scaleWidth = ui.scaleWidthCheckBox->isChecked();
  settings.width = ui.widthSpinBox->value();
  settings.scaleHeight = ui.scaleHeightCheckBox->isChecked();
  settings.height = ui.heightSpinBox->value();
  settings.keepRatio = ui.keepRatioCheckBox->isChecked();
}

void RqtImagePublisher::rescaleImageLabel()
{
  if (image_qpix.isNull())
      return;

  int w = ui.imageView->width();
  int h = ui.imageView->height();
  ui.imageView->setPixmap(image_qpix.scaled(w,h,Qt::KeepAspectRatio));
}

void RqtImagePublisher::applySettings()
{
  publishingTimer.stop();
  image_pub.shutdown();
  bool latched = settings.publishLatched && !settings.publishContinuously && !settings.rotateImages;
  image_pub = imt->advertise(settings.imageTopic.toStdString(), 1, latched);

  if (image_loaded)
  {
    generateCvBridgeImage();
    generatePixmap();
  }

  if (settings.rotateImages)
  {
    ui.publishButton->setText("Start slideshow");
    publishingTimer.setInterval((int)(1000.0 / settings.rotationFrequency));
  }
  else if (settings.publishContinuously)
  {
    ui.publishButton->setText("Start publishing");
    publishingTimer.setInterval((int)(1000.0 / settings.publishingFrequency));
  }
  else
  {
    ui.publishButton->setText("Publish");
  }

}

} // namespace rqt_image_publisher

PLUGINLIB_EXPORT_CLASS(rqt_image_publisher::RqtImagePublisher, rqt_gui_cpp::Plugin)
//PLUGINLIB_DECLARE_CLASS(rqt_image_publisher, RqtImagePublisher, rqt_image_publisher::RqtImagePublisher, rqt_gui_cpp::Plugin)
