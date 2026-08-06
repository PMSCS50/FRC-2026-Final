package frc.robot.util.pathfinding.events;

import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.*;

/**
 * Extends EventScheduler to handle events using WPILIB CommandScheduler.
 * This is done so that we dont have to define requirements in ShinPathfindingCommand
 * which allows us to get all the requirements dynamically as new paths are found
 */
public class EventSupervisor extends EventScheduler{
  private static final EventLoop eventLoop = EventScheduler.getEventLoop();

  private final Map<Command, Boolean> eventCommands;
  private final Queue<Event> upcomingEvents;

  private final Set<Command> scheduledCommands;

  /** Create a new EventSupervisor */
  public EventSupervisor() {
    this.eventCommands = new HashMap<>();
    this.upcomingEvents =
        new PriorityQueue<>(Comparator.comparingDouble(Event::getTimestampSeconds));
    this.scheduledCommands = new HashSet<>();
  }

  /**
   * Initialize the EventSupervisor for the given trajectory. This should be called from the
   * initialize method of the command running this scheduler.
   *
   * @param trajectory The trajectory this scheduler should handle events for
   */
  @Override
  public void initialize(PathPlannerTrajectory trajectory) {
    eventCommands.clear();
    upcomingEvents.clear();
    scheduledCommands.clear();
    upcomingEvents.addAll(trajectory.getEvents());
  }

  /**
   * Run the scheduler. This should be called from the execute method of the command running this
   * scheduler.
   *
   * @param time The current time along the trajectory
   */
  @Override
  public void execute(double time) {
    // Check for events that should be handled this loop
    while (!upcomingEvents.isEmpty() && time >= upcomingEvents.peek().getTimestampSeconds()) {
      upcomingEvents.poll().handleEvent(this);
     
    }

    // // Run currently running commands
    // for (var entry : eventCommands.entrySet()) {
    //   if (!entry.getValue()) {
    //     continue;
    //   }

    //   entry.getKey().execute();
    //   if (entry.getKey().isFinished()) {
    //     entry.getKey().end(false);
    //     eventCommands.put(entry.getKey(), false);
    //   }
    // }

    eventLoop.poll();
  }

  /**
   * End commands currently/events currently being handled by this scheduler. This should be called
   * from the end method of the command running this scheduler.
   */
   @Override
    public void end() {
        for (Command c : scheduledCommands) {
            CommandScheduler.getInstance().cancel(c);
        }

        scheduledCommands.clear();

        for (Event e : upcomingEvents) {
            e.cancelEvent(this);
        }
        upcomingEvents.clear();
    }

  /**
   * Get the event loop used to poll global event triggers
   *
   * @return Event loop that polls global event triggers
   */
  protected static EventLoop getEventLoop() {
    return eventLoop;
  }

  public static Set<Subsystem> getAllSchedulerRequirements(PathPlannerPath path) {
    return EventScheduler.getSchedulerRequirements(path);
  }

  /**
   * Schedule a command on this scheduler. This will cancel other commands that share requirements
   * with the given command.
   *
   * @param command The command to schedule
   */
  @Override
  protected void scheduleCommand(Command command) {
    scheduledCommands.add(command);
    CommandScheduler.getInstance().schedule(command);
  }

  /**
   * Cancel a command on this scheduler.
   *
   * @param command The command to cancel
   */
  @Override
  protected void cancelCommand(Command command) {
      if (scheduledCommands.remove(command)) {
          CommandScheduler.getInstance().cancel(command);
      }
  }
}