#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <subsystems/DriveTrain.h>
#include <subsystems/Localizer.h>
#include <subsystems/FieldMap.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/Sendable.h>

class FieldMapDisplay : public frc2::SubsystemBase
{
    public: 
    FieldMapDisplay(std::shared_ptr<Drivetrain> driveTrain, std::shared_ptr<Localizer> localizer, std::shared_ptr<FieldMap> fieldmap);
    void Periodic() override;


    private:
    frc::Field2d _field;
    std::shared_ptr<Drivetrain> _driveTrain;
    std::shared_ptr<Localizer> _localizer;    
    std::shared_ptr<FieldMap> _fieldMap;
    frc::Pose3d _robotpose;
};