#include "ui_mainwindow.h"
#include "../include/project_pkg/mainwindow.h"
QString situation;


MainWindow::MainWindow(int argc, char*argv[], QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));
    spinner = NULL;
    ros::init(argc, argv, "Project");
    if ( ros::master::check() )
    {
        ui->lblConnect->setText("Connect to Ros!");
        spinner = new ros::AsyncSpinner(2);

    }
    else{
        ui->lblConnect->setText("Failed to Connect to Ros!");
        this->setEnabled(false);
    }
    ui->dispositivo2GpBox->setVisible(false);
    ui->dispositivoGpBox->setVisible(false);
    ui->teclasGpBox->setVisible(false);
    ui->vModeCB_2->setVisible(false);
    ui->vModeLbl_2->setVisible(false);
    ui->vModeCB->setVisible(false);
    ui->vModeLbl->setVisible(false);

    glViewer = new GLWidget(ui->viewFrame);
    glViewer->resize(630,400);
    glViewer2 = new GLWidget(ui->viewFrame_2);
    glViewer2->resize(620,400);
}

MainWindow::~MainWindow()
{
    if(ros::isStarted()) {
        if(spinner)
        {
            spinner->stop();
        }
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
    delete ui;
}

//void *MainWindow::labelsThreadFunc(void *arg){
//  MainWindow *w;
//  w = (MainWindow*)arg;
//  w->atualiza_labels();
//  pthread_exit(0);
//}

//void MainWindow::atualiza_labels(){
//  if(glViewer->GetStatus() == 1){
//    this->ui->angRGBLbl->setText(QString::number(glViewer->freenect_angle));
//    this->ui->modLbl->setText(glViewer->modoVideo);
//  }
//  if(this->glViewer2->GetStatus() == 2){
//    QString aux_aux2;
//    aux_aux2.clear();
//    int index;
//    index = (this->ui->angSpin->value()+135)/0.35;
//    aux_aux2.append(QString::number(this->glViewer2->laser->laserData[index].distancia));
//    this->ui->distLbl->setText(aux_aux2);
//  }
//}

void MainWindow::on_actionExit_triggered()
{
    this->close();
    ros::shutdown();
}

// ----------------------------------------------------------------------------------------------------------------------------------

void MainWindow::LaserInit(int viewer){
    ros::NodeHandle laser_nodeHandle;
    if (viewer == 0)
    {
        glViewer->LaserInit(laser_nodeHandle);
        this->ui->dispositivoLbl->setText("Laser");
        this->ui->distLbl->clear();
        this->ui->dispositivo2GpBox->setVisible(true);
    }
    else
    {
        glViewer2->LaserInit(laser_nodeHandle);
        this->ui->dispositivoLbl_2->setText("Laser");
        this->ui->distLbl->clear();
        this->ui->dispositivo2GpBox->setVisible(true);
    }
    spinner->start();
}

void MainWindow::ZedInit(int viewer){
    if (viewer == 0)
    {
        glViewer->ZedInit();
        this->ui->dispositivoLbl->setText("Zed");
        this->ui->distLbl->clear();
        this->ui->dispositivo2GpBox->setVisible(true);
        this->ui->vModeCB->setVisible(true);
        this->ui->vModeLbl->setVisible(true);
    }
    else
    {
        glViewer2->ZedInit();
        this->ui->dispositivoLbl_2->setText("Zed");
        this->ui->distLbl->clear();
        this->ui->dispositivo2GpBox->setVisible(true);
        this->ui->vModeCB_2->setVisible(true);
        this->ui->vModeLbl_2->setVisible(true);

    }
    spinner->start();
}

void MainWindow::AstraInit(int viewer){
    ros::NodeHandle astra_nodeHandle;
    if (viewer == 0){
        glViewer->AstraInit(astra_nodeHandle);
        glViewer->SetStatus(3);
        this->ui->dispositivoLbl->setText("Astra");
        this->ui->vModeCB->setVisible(true);
        this->ui->vModeLbl->setVisible(true);

    }
    else
    {
        glViewer2->AstraInit(astra_nodeHandle);
        glViewer2->SetStatus(3);
        this->ui->dispositivoLbl_2->setText("Astra");
        this->ui->vModeCB_2->setVisible(true);
        this->ui->vModeLbl_2->setVisible(true);

    }
    spinner->start();
}

void MainWindow::ImuInit(int viewer){
    ros::NodeHandle imu_nodeHandle;
    if (viewer == 0){
        glViewer->ImuInit(imu_nodeHandle);
        glViewer->SetStatus(5);
        this->ui->dispositivoLbl->setText("Imu");
    }
    else
    {
        glViewer2->ImuInit(imu_nodeHandle);
        glViewer2->SetStatus(5);
        this->ui->dispositivoLbl_2->setText("Imu");
    }
    spinner->start();
}

void MainWindow::KinectInit(int viewer){
    ros::NodeHandle kinect_nodeHandle;
    int aux;
    if (viewer == 0)
    {
        aux = glViewer->KinectStart();
        if(aux > 0){
            situation.append("Dispositivo RGBD conectado com sucesso!");
            this->ui->dispositivoGpBox->setVisible(true);
            this->ui->teclasGpBox->setVisible(true);
            ui->dispositivoLbl->setText("Kinect");
            glViewer->SetStatus(1);
            this->ui->vModeCB->setVisible(true);
            this->ui->vModeLbl->setVisible(true);
        }else{
            situation.append("Nenhum dispositivo RGBD conectado!");
        }
    }
    else
    {
        aux = glViewer2->KinectStart();
        if(aux > 0){
            situation.append("Dispositivo RGBD conectado com sucesso!");
            this->ui->dispositivoGpBox->setVisible(true);
            this->ui->teclasGpBox->setVisible(true);
            ui->dispositivoLbl_2->setText("Kinect");
            glViewer2->SetStatus(1);
            this->ui->vModeCB_2->setVisible(true);
            this->ui->vModeLbl_2->setVisible(true);
        }else{
            situation.append("Nenhum dispositivo RGBD conectado!");
        }
    }

    spinner->start();
}

// ----------------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------

void MainWindow::stopp()
{
    ros::shutdown();
}

//Seleção do Dispositivo

void MainWindow::on_dispositivosCB_activated(int index)
{
    situation.clear();
    int aux;

    if(spinner && !spinner->canStart())
        spinner->stop();

    if(index==0)
    {
        situation.append("Nenhum Dispositivo Selecionado!");
        this->ui->dispositivoGpBox->setVisible(false);
        this->ui->dispositivo2GpBox->setVisible(false);
        this->ui->teclasGpBox->setVisible(false);
    }
    if(index==1)
        LaserInit(0);
    if(index==2)
        KinectInit(0);
    if(index==3)
        ImuInit(0);
    if(index==4)
        AstraInit(0);
    if(index==5)
        ZedInit(0);
        
    ui->txtBox->setText(situation);
  
}

void MainWindow::on_dispositivosCB_currentIndexChanged(const QString &arg1)
{
    situation.clear();
    this->ui->distLbl->clear();
    this->ui->angSpin->setValue(0.00);
    this->ui->dispositivo2GpBox->setVisible(false);
    this->ui->dispositivoGpBox->setVisible(false);
    this->ui->teclasGpBox->setVisible(false);
    this->ui->dispositivoLbl->clear();
    this->ui->angRGBLbl->clear();
    this->ui->modLbl->clear();
    this->glViewer->Stop();
    this->ui->txtBox->setText(situation);
    this->ui->vModeCB->setVisible(false);
    this->ui->vModeLbl->setVisible(false);
}

void MainWindow::on_angSpin_valueChanged(double arg1)
{
    QString aux_aux;
    aux_aux.clear();
    int id;
    id = (arg1+135)/0.35;
    aux_aux.append(QString::number(glViewer2->laser->laserData[id].distancia));
    this->ui->distLbl->setText(aux_aux);
}

void MainWindow::on_vModeCB_currentIndexChanged(int index)
{
	glViewer->ChangeMod(index);
}

void MainWindow::on_dispositivosCB_2_activated(int index)
{
    situation.clear();
    int aux;

    if(spinner && !spinner->canStart())
        spinner->stop();

    if(index==0)
    {
        situation.append("Nenhum Dispositivo Selecionado!");
        this->ui->dispositivoGpBox->setVisible(false);
        this->ui->dispositivo2GpBox->setVisible(false);
        this->ui->teclasGpBox->setVisible(false);
    }
    if(index==1)
        LaserInit(1);
    if(index==2)
        KinectInit(1);
    if(index==3)
        ImuInit(1);
    if(index==4)
        AstraInit(1);
    if(index==5)
        ZedInit(1);
        
    ui->txtBox->setText(situation);
  
}

void MainWindow::on_dispositivosCB_2_currentIndexChanged(int index)
{
    situation.clear();
    this->ui->distLbl->clear();
    this->ui->angSpin->setValue(0.00);
    this->ui->dispositivo2GpBox->setVisible(false);
    this->ui->dispositivoGpBox->setVisible(false);
    this->ui->teclasGpBox->setVisible(false);
    this->ui->dispositivoLbl_2->clear();
    this->ui->angRGBLbl->clear();
    this->ui->modLbl->clear(); 
    this->ui->vModeCB_2->setVisible(false);
    this->ui->vModeLbl_2->setVisible(false);
    this->glViewer2->Stop();
    this->ui->txtBox->setText(situation);
}

void MainWindow::on_vModeCB_2_currentIndexChanged(int index)
{
	glViewer2->ChangeMod(index);
}

void MainWindow::on_pushButton_clicked()
{
    glViewer->saveFiles();
}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
    if(arg1 == 0)
        std::cout << "Syncronization OFF"<< std::endl;
    else{
        std::cout << "Syncronization ON"<< std::endl;
        glViewer->Syncronize(spinner);
    }
}
