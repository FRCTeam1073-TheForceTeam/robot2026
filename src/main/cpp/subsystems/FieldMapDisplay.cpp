#include <subsystems/FieldMapDisplay.h>

FieldMapDisplay::FieldMapDisplay(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<Localizer> localizer, std::shared_ptr<FieldMap> fieldmap):
    _driveTrain(driveTrain),
    _localizer(localizer),
    _fieldMap(fieldmap)
{}
   
void FieldMapDisplay::Periodic() {
    _field.SetRobotPose(_localizer->getPose());
    frc::SmartDashboard::PutData("Field", &_field);
}