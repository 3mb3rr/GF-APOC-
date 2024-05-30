package org.firstinspires.ftc.teamcode.common.util.datalogging;

public class Datalog
{
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
    public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
    public Datalogger.GenericField yaw          = new Datalogger.GenericField("Yaw");
    public Datalogger.GenericField pitch        = new Datalogger.GenericField("Pitch");
    public Datalogger.GenericField roll         = new Datalogger.GenericField("Roll");
    public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

    public Datalog(String name)
    {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        opModeStatus,
                        loopCounter,
                        yaw,
                        pitch,
                        roll,
                        battery
                )
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}
