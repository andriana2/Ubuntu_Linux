#include "../include/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    turtlesim_started_ = false;
    setupConnections();
    initROS();

}

MainWindow::~MainWindow() {
    delete ui;
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}


void MainWindow::setupConnections() {
    connect(ui->button_start, &QPushButton::clicked, this, &MainWindow::startTurtlesim);//
    connect(ui->button_stop, &QPushButton::clicked, this, &MainWindow::stopTurtlesim);//
    
    //connect(ui->button_connect, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);

}

void MainWindow::initROS() {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("intento3");
    publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
}


// void MainWindow::onPublishTimer() {
//     if (!rclcpp::ok()) {
//         QMessageBox::critical(this, "ROS Error", "ROS is not running!");
//         publishTimer_->stop();
//         return;
//     }

//     auto message = std_msgs::msg::String();
//     message.data = "Hello, ROS 2! " + std::to_string(count_++);
//     publisher_->publish(message);

//     ui->view_logging->addItem(QString::fromStdString("Published: " + message.data));
//     rclcpp::spin_some(node_);
// }

/////////////
void MainWindow::startTurtlesim()
{
    if (!turtlesim_started_) {
        std::system("ros2 run turtlesim turtlesim_node &");  // Ejecuta turtlesim en segundo plano
        turtlesim_started_ = true;
    }
}

void MainWindow::stopTurtlesim()
{
    if (turtlesim_started_) {
        std::system("pkill turtlesim_node");  // Detiene el nodo de turtlesim
        turtlesim_started_ = false;
    }
}