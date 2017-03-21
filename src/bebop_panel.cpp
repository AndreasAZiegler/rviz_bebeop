/**
 * @file bebop_panel.cpp
 * @brief Bebop control panel for rviz
 * @author Andreas Ziegler
 */

// Includes
#include <stdio.h>
#include <math.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include "bebop_panel.h"

namespace rviz_bebop {

	/**
	 * @brief Constructor, Builds up GUI and initialize ROS
	 * @param parent Pointeer to the parent widget
	 */
	BebopPanel::BebopPanel(QWidget* parent)
		: rviz::Panel(parent)
	{
		// Namespace field
		QHBoxLayout* namespace_layout = new QHBoxLayout;
		namespace_layout->addWidget(new QLabel("Bebop Namespace:"));
		output_namespace_editor_ = new QLineEdit;
		namespace_layout->addWidget(output_namespace_editor_);

		// Takeoff, Land and Reset Bebop control buttons
		QHBoxLayout* control_layout = new QHBoxLayout;
		takeoff_button_ = new QPushButton(QString("Takeoff"));
		control_layout->addWidget(takeoff_button_);
		land_button_ = new QPushButton(QString("Land"));
		control_layout->addWidget(land_button_);
		reset_button_ = new QPushButton(QString("Reset"));
		control_layout->addWidget(reset_button_);

		// Move by Bebop control
		QVBoxLayout* move_bebop_layout = new QVBoxLayout;
		QHBoxLayout* move_bebop_d_layout = new QHBoxLayout;
		move_bebop_d_layout->addWidget(new QLabel("dX [m]"));
		move_bebop_dx_editor_ = new QLineEdit;
		move_bebop_d_layout->addWidget(move_bebop_dx_editor_);
		move_bebop_d_layout->addWidget(new QLabel("dY [m]"));
		move_bebop_dy_editor_ = new QLineEdit;
		move_bebop_d_layout->addWidget(move_bebop_dy_editor_);
		move_bebop_d_layout->addWidget(new QLabel("dZ [m]"));
		move_bebop_dz_editor_ = new QLineEdit;
		move_bebop_d_layout->addWidget(move_bebop_dz_editor_);
		move_bebop_d_layout->addWidget(new QLabel("dPsi [deg]"));
		move_bebop_dpsi_editor_ = new QLineEdit;
		move_bebop_d_layout->addWidget(move_bebop_dpsi_editor_);
		move_bebop_layout->addLayout(move_bebop_d_layout);
		move_bebop_button_ = new QPushButton(QString("Move Bebop"));
		move_bebop_layout->addWidget(move_bebop_button_);

		// Lay out the topic field above the bebop control.
		QVBoxLayout* layout = new QVBoxLayout;
		layout->addLayout(namespace_layout);
		layout->addLayout(control_layout);
		layout->addLayout(move_bebop_layout);
		setLayout(layout);

		// Next we make signal/slot connections.
		connect(output_namespace_editor_, SIGNAL(editingFinished()), this, SLOT(updateNamespace()));
		connect(takeoff_button_, SIGNAL(clicked()), this, SLOT(sendTakeoffBebop()));
		connect(land_button_, SIGNAL(clicked()), this, SLOT(sendLandBebop()));
		connect(reset_button_, SIGNAL(clicked()), this, SLOT(sendResetBebop()));
		connect(move_bebop_button_, SIGNAL(clicked()), this, SLOT(sendMoveByBebop()));

		// Advertise ROS topics
		takeoff_publisher_ = nh_.advertise<std_msgs::Empty>(bebop_namespace_.toStdString() + "/takeoff", 1);
		land_publisher_ = nh_.advertise<std_msgs::Empty>(bebop_namespace_.toStdString() + "/land", 1);
		reset_publisher_ = nh_.advertise<std_msgs::Empty>(bebop_namespace_.toStdString() + "/reset", 1);
		move_publisher_ = nh_.advertise<geometry_msgs::Twist>(bebop_namespace_.toStdString() + "/cmd_move_by", 1 );
	}

	/**
	 * @brief Calls setNamespace with the value in the according QLineEdit field
	 */
	void BebopPanel::updateNamespace() {
		setNamespace(output_namespace_editor_->text());
	}

	/**
	 * @brief Sends a takeoff topic to the Bebop.
	 */
	void BebopPanel::sendTakeoffBebop() {
		std_msgs::Empty msg;
		takeoff_publisher_.publish(msg);
	}

	/**
	 * @brief Sends a land topic to the Bebop.
	 */
	void BebopPanel::sendLandBebop() {
		std_msgs::Empty msg;
		land_publisher_.publish(msg);
	}

	/**
	 * @brief Sends a reset topic to the Bebop.
	 */
	void BebopPanel::sendResetBebop() {
		std_msgs::Empty msg;
		reset_publisher_.publish(msg);
	}

	/**
	 * @brief Sends a cmd_move_by topic to the Bebop with dx, dy, dz and dPsi given in the according QLineEdit fields.
	 */
	void BebopPanel::sendMoveByBebop() {
		geometry_msgs::Twist msg;
		msg.linear.x = move_bebop_dx_editor_->text().toDouble();
		msg.linear.y = move_bebop_dy_editor_->text().toDouble();
		msg.linear.z = move_bebop_dz_editor_->text().toDouble();
		msg.angular.z = move_bebop_dpsi_editor_->text().toDouble()*M_PI/180;
		move_publisher_.publish(msg);
	}

	/**
	 * @brief Sets the name space of the Bebop if it changed and updates the publishers.
	 * @param new_namespace the new name space
	 */
	void BebopPanel::setNamespace(const QString& new_namespace) {
		// Only take action if the name has changed.
		if(new_namespace != bebop_namespace_) {
			bebop_namespace_ = new_namespace;

			// Shut down the publishers
			takeoff_publisher_.shutdown();
			land_publisher_.shutdown();
			reset_publisher_.shutdown();
			move_publisher_.shutdown();
			// Advertise the publishers with the new name space
			takeoff_publisher_ = nh_.advertise<std_msgs::Empty>(bebop_namespace_.toStdString() + "/takeoff", 1);
			land_publisher_ = nh_.advertise<std_msgs::Empty>(bebop_namespace_.toStdString() + "/land", 1);
			reset_publisher_ = nh_.advertise<std_msgs::Empty>(bebop_namespace_.toStdString() + "/reset", 1);
			move_publisher_ = nh_.advertise<geometry_msgs::Twist>(bebop_namespace_.toStdString() + "/cmd_move_by", 1 );
		}
	}

	/**
	 * @brief Save all configuration data from this panel to the given Config object.
	 * @param config Config object
	 */
	void BebopPanel::save(rviz::Config config) const {
		rviz::Panel::save( config );
		config.mapSetValue( "Topic", bebop_namespace_ );
	}

	/**
	 * @brief Load all configuration data for this panel from the given Config object.
	 * @param config Config object
	 */
	void BebopPanel::load(const rviz::Config& config) {
		rviz::Panel::load(config);
		QString topic;
		if( config.mapGetString("Topic", &topic)) {
			output_namespace_editor_->setText(topic);
			updateNamespace();
		}
	}
} // end namespace rviz_bebop

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_bebop::BebopPanel,rviz::Panel)
