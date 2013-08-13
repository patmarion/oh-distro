#include "plotwidget.h"
#include "plot.h"
#include "lcmthread.h"
#include "signalhandler.h"
#include "setscaledialog.h"
#include "selectsignaldialog.h"
#include "signaldescription.h"

#include <qwt_scale_engine.h>
#include <qlabel.h>
#include <qlayout.h>
#include <QDoubleSpinBox>
#include <QMenu>
#include <QAction>
#include <QLabel>
#include <QInputDialog>
#include <QDialogButtonBox>
#include <QListWidget>

PlotWidget::PlotWidget(LCMThread* lcmThread, QWidget *parent):
    QWidget(parent),
    mLCMThread(lcmThread)
{

  d_plot = new Plot(this);

  mColors << Qt::green
          << Qt::red
          << Qt::blue
          << Qt::yellow
          << Qt::cyan
          << Qt::darkCyan
          << Qt::black;

  QDoubleSpinBox* timeWindowSpin = new QDoubleSpinBox;
  timeWindowSpin->setSingleStep(0.1);

  QDoubleSpinBox* yScaleSpin = new QDoubleSpinBox;
  yScaleSpin->setSingleStep(0.1);

  QVBoxLayout* vLayout1 = new QVBoxLayout();
  vLayout1->addWidget(timeWindowSpin);
  vLayout1->addWidget(new QLabel("Time Window [s]"));
  vLayout1->addWidget(yScaleSpin);
  vLayout1->addWidget(new QLabel("Y Scale [+/-]"));


  mSignalListWidget = new QListWidget(this);
  vLayout1->addWidget(mSignalListWidget);
  
  vLayout1->addStretch(10);


  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->addWidget(d_plot, 10);
  layout->addLayout(vLayout1);

  connect(timeWindowSpin, SIGNAL(valueChanged(double)),
          d_plot, SLOT(setTimeWindow(double)));
  timeWindowSpin->setValue(10.0);

  connect(yScaleSpin, SIGNAL(valueChanged(double)),
          d_plot, SLOT(setYScale(double)));
  yScaleSpin->setValue(10.0);

  this->setContextMenuPolicy(Qt::CustomContextMenu);
  this->connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),
      SLOT(onShowContextMenu(const QPoint&)));

  mSignalListWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->connect(mSignalListWidget, SIGNAL(customContextMenuRequested(const QPoint&)),
      SLOT(onShowSignalContextMenu(const QPoint&)));

  this->connect(mSignalListWidget, SIGNAL(itemChanged(QListWidgetItem *)), SLOT(onSignalListItemChanged(QListWidgetItem*)));

  this->start();
}

void PlotWidget::onShowContextMenu(const QPoint& pos)
{

  QPoint globalPos = this->mapToGlobal(pos);
  // for QAbstractScrollArea and derived classes you would use:
  // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); 

  QMenu myMenu;
  myMenu.addAction("Add signal");
  myMenu.addAction("Set Y axis scale");
  myMenu.addSeparator();
  myMenu.addAction("Remove plot");

  QAction* selectedItem = myMenu.exec(globalPos);
  if (!selectedItem)
  {
    return;
  }

  QString selectedAction = selectedItem->text();

  if (selectedAction == "Remove plot")
  {
    emit this->removePlotRequested(this);
  }
  else if (selectedAction == "Add signal")
  {
    emit this->addSignalRequested(this);
  }
  else if (selectedAction == "Set Y axis scale")
  {

    SetScaleDialog dialog(this);
    QwtInterval axisInterval = d_plot->axisInterval(QwtPlot::yLeft);
    dialog.setUpper(axisInterval.maxValue());
    dialog.setLower(axisInterval.minValue());

    int result = dialog.exec();
    if (result == QDialog::Accepted)
    {
      d_plot->setAxisScale(QwtPlot::yLeft, dialog.lower(), dialog.upper());
    }
  }
}

void PlotWidget::onShowSignalContextMenu(const QPoint& pos)
{

  if (mSignals.empty())
  {
    return;
  }

  QPoint globalPos = mSignalListWidget->mapToGlobal(pos);
  // for QAbstractScrollArea and derived classes you would use:
  // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); 

  QMenu myMenu;
  myMenu.addAction("Remove signal");

  QAction* selectedItem = myMenu.exec(globalPos);
  if (!selectedItem)
  {
    return;
  }

  QString selectedAction = selectedItem->text();

  if (selectedAction == "Remove signal")
  {
    QListWidgetItem* signalItem = mSignalListWidget->currentItem();

    SignalHandler* signalHandler = signalItem->data(Qt::UserRole).value<SignalHandler*>();

    mLCMThread->removeSignalHandler(signalHandler);
    d_plot->removeSignal(signalHandler->signalData());
    mSignals.removeAll(signalHandler);

    delete signalItem;
    delete signalHandler;
  }

}

void PlotWidget::onSignalListItemChanged(QListWidgetItem* item)
{
  SignalHandler* signalHandler = item->data(Qt::UserRole).value<SignalHandler*>();
  bool checked = (item->checkState() == Qt::Checked);
  d_plot->setSignalVisible(signalHandler->signalData(), checked);
}

void PlotWidget::start()
{
  d_plot->start();
}

void PlotWidget::stop()
{
  d_plot->stop();
}

void PlotWidget::addSignal(const QMap<QString, QVariant>& signalSettings)
{
  SignalDescription desc;
  desc.mChannel = signalSettings.value("channel").toString();
  desc.mMessageType = signalSettings.value("messageType").toString();
  desc.mFieldName = signalSettings.value("fieldName").toString();
  desc.mArrayKeys = signalSettings.value("arrayKeys").toStringList();

  SignalHandler* signalHandler = SignalHandlerFactory::instance().createHandler(&desc);

  if (signalHandler)
  {
    printf("adding signal: %s\n", qPrintable(signalHandler->description()));
  }
  else
  {
    printf("failed to create signal from signal settings.\n");
  }

  this->addSignal(signalHandler);
}

Q_DECLARE_METATYPE(SignalHandler*);

void PlotWidget::addSignal(SignalHandler* signalHandler)
{
  if (!signalHandler)
  {
    return;
  }

  int signalColorIndex = mSignals.size() % mColors.size();
  mLCMThread->addSignalHandler(signalHandler);
  mSignals.append(signalHandler);
  d_plot->addSignal(signalHandler->signalData(), mColors[signalColorIndex]);

  QString signalDescription = QString("%2 [%1]").arg(signalHandler->channel()).arg(signalHandler->description().split(".").back());
  QListWidgetItem* signalItem = new QListWidgetItem(signalDescription);

  QPixmap pixmap(24, 24);
  pixmap.fill(mColors[signalColorIndex]);
  signalItem->setIcon(QIcon(pixmap));

  signalItem->setData(Qt::UserRole, qVariantFromValue(signalHandler));
  signalItem->setData(Qt::CheckStateRole, Qt::Checked);

  mSignalListWidget->addItem(signalItem);
}


void PlotWidget::loadSettings(const QMap<QString, QVariant>& plotSettings)
{
  QList<QVariant> signalList = plotSettings.value("signals").toList();
  foreach (const QVariant& signalVariant, signalList)
  {
    this->addSignal(signalVariant.toMap());
  }

  double timeWindow = plotSettings.value("timeWindow", QVariant(10.0)).toDouble();
  double ymin = plotSettings.value("ymin", QVariant(-10.0)).toDouble();
  double ymax = plotSettings.value("ymax", QVariant(10.0)).toDouble();
  d_plot->setAxisScale(QwtPlot::yLeft, ymin, ymax);
  d_plot->setTimeWindow(timeWindow);
}

QMap<QString, QVariant> PlotWidget::saveSettings()
{
  QMap<QString, QVariant> settings;

  settings["ymin"] = d_plot->axisInterval(QwtPlot::yLeft).minValue();
  settings["ymax"] = d_plot->axisInterval(QwtPlot::yLeft).maxValue();
  settings["timeWindow"] = d_plot->timeWindow();

  QList<QVariant> signalSettings;
  foreach (SignalHandler* signalHandler, mSignals)
  {
    signalSettings.append(this->saveSignalSettings(signalHandler)); 
  }

  settings["signals"] = signalSettings;
  return settings;
}


QMap<QString, QVariant> PlotWidget::saveSignalSettings(SignalHandler* signalHandler)
{
  QMap<QString, QVariant> settings;

  SignalDescription* signalDescription = signalHandler->signalDescription();

  settings["channel"] = signalDescription->mChannel;
  settings["messageType"] = signalDescription->mMessageType;
  settings["fieldName"] = signalDescription->mFieldName;
  settings["arrayKeys"] = QVariant(signalDescription->mArrayKeys);

  return settings;
}
