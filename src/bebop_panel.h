/**
 * @file bebop_panel.h
 * @brief Bebop control panel for rviz
 * @author Andreas Ziegler
 */

#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

// Includes
#include <ros/ros.h>

#include <rviz/panel.h>

// Forward class declarations
class QLineEdit;
class QPushButton;

namespace rviz_bebop {

	/**
	 * @brief The BebopPanel class
	 */
	class BebopPanel: public rviz::Panel
	{
	Q_OBJECT

	public:
		/**
		 * @brief Constructor, Builds up GUI and initialize ROS
		 * @param parent Pointeer to the parent widget
		 */
		BebopPanel(QWidget* parent = 0);

		/**
		 * @brief Save all configuration data from this panel to the given Config object.
		 * @param config Config object
		 */
		virtual void load(const rviz::Config& config);

		/**
		 * @brief Load all configuration data for this panel from the given Config object.
		 * @param config Config object
		 */
		virtual void save(rviz::Config config) const;

	public Q_SLOTS:
		/**
		 * @brief Sets the name space of the Bebop if it changed and updates the publishers.
		 * @param new_namespace the new name space
		 */
		void setNamespace(const QString& new_namespace );

	protected Q_SLOTS:
		/**
		 * @brief Calls setNamespace with the value in the according QLineEdit field
		 */
		void updateNamespace();

		/**
		 * @brief Sends a takeoff topic to the Bebop.
		 */
		void sendTakeoffBebop();

		/**
		 * @brief Sends a land topic to the Bebop.
		 */
		void sendLandBebop();

		/**
		 * @brief Sends a reset topic to the Bebop.
		 */
		void sendResetBebop();

		/**
		 * @brief Sends a cmd_move_by topic to the Bebop with dx, dy, dz and dPsi given in the according QLineEdit fields.
		 */
		void sendMoveByBebop();

	protected:
		QLineEdit* output_namespace_editor_;    /**< QLineEdit for the Bebop ROS name space */

		QPushButton* takeoff_button_;						/**< QPushButton to send a takeoff topic to the Bebop */
		QPushButton* land_button_;							/**< QPushButton to send a land topic to the Bebop */
		QPushButton* reset_button_;							/**< QPushButton to send a reset topic to the Bebop */

		QLineEdit* move_bebop_dx_editor_;				/**< QLineEdit for the change in X direction the Bebop has to move */
		QLineEdit* move_bebop_dy_editor_;				/**< QLineEdit for the change in Y direction the Bebop has to move */
		QLineEdit* move_bebop_dz_editor_;				/**< QLineEdit for the change in Z direction the Bebop has to move */
		QLineEdit* move_bebop_dpsi_editor_;			/**< QLineEdit for the change in rotation the Bebop has to move */
		QPushButton* move_bebop_button_;				/**< QPushBotton to send a cmd_mov_by topic to the Bebop. */

	QString bebop_namespace_;									/**< The current name space of the bebop */

  ros::Publisher takeoff_publisher_;				/**< Publisher for the takeoff topic */
  ros::Publisher land_publisher_;						/**< Publisher for the land topic */
  ros::Publisher reset_publisher_;					/**< Publisher for the reset topic */
  ros::Publisher move_publisher_;						/**< Publisher for the move topic */

  ros::NodeHandle nh_;											/**< The ROS node handle */
};

} // end namespace rviz_bebop

#endif // BEBOP_PANEL_H
