# Don't import stuff from the package here, it'll cause a circular import


from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING, Any, Callable, Set

from typing_extensions import Self, TypeAlias

if TYPE_CHECKING:
    from .instantcommand import InstantCommand
    from .subsystem import Subsystem
    from .parallelracegroup import ParallelRaceGroup
    from .sequentialcommandgroup import SequentialCommandGroup
    from .paralleldeadlinegroup import ParallelDeadlineGroup
    from .parallelcommandgroup import ParallelCommandGroup
    from .perpetualcommand import PerpetualCommand
    from .repeatcommand import RepeatCommand
    from .proxycommand import ProxyCommand
    from .conditionalcommand import ConditionalCommand
    from .wrappercommand import WrapperCommand

from wpiutil import Sendable, SendableRegistry, SendableBuilder


class InterruptionBehavior(Enum):
    """
    An enum describing the command's behavior when another command with a shared requirement is
    scheduled.
    """

    kCancelIncoming = 0
    """
    This command ends, #end(boolean) end(true) is called, and the incoming command is
    scheduled normally.

    This is the default behavior.
    """

    kCancelSelf = 1
    """ This command continues, and the incoming command is not scheduled."""


class Command(Sendable):
    """
    A state machine representing a complete action to be performed by the robot. Commands are run by
    the CommandScheduler, and can be composed into CommandGroups to allow users to build
    complicated multistep actions without the need to roll the state machine logic themselves.

    Commands are run synchronously from the main robot loop; no multithreading is used, unless
    specified explicitly from the command implementation.

    This class is provided by the NewCommands VendorDep
    """

    InterruptionBehavior: TypeAlias = (
        InterruptionBehavior  # type alias for 2023 location
    )

    requirements: Set[Subsystem]

    def __new__(cls, *args, **kwargs) -> Self:
        instance = super().__new__(
            cls,
        )
        super().__init__(instance)
        SendableRegistry.add(instance, cls.__name__)
        instance.requirements = set()
        return instance

    def __init__(self):
        pass

    def initialize(self):
        """The initial subroutine of a command. Called once when the command is initially scheduled."""
        pass

    def execute(self):
        """The main body of a command. Called repeatedly while the command is scheduled."""
        pass

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its end()
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        return False

    def end(self, interrupted: bool):
        """
        The action to take when the command ends. Called when either the command finishes normally, or
        when it interrupted/canceled.

        Do not schedule commands here that share requirements with this command. Use {@link
        #andThen(Command...)} instead.

        :param interrupted: whether the command was interrupted/canceled
        """
        pass

    def getRequirements(self) -> Set[Subsystem]:
        """
        Specifies the set of subsystems used by this command. Two commands cannot use the same
        subsystem at the same time. If another command is scheduled that shares a requirement, {@link
        #getInterruptionBehavior()} will be checked and followed. If no subsystems are required, return
        an empty set.

        Note: it is recommended that user implementations contain the requirements as a field, and
        return that field here, rather than allocating a new set every time this is called.

        :returns: the set of subsystems that are required
        """
        return self.requirements

    def addRequirements(self, *requirements: Subsystem):
        """
        Adds the specified subsystems to the requirements of the command. The scheduler will prevent
        two commands that require the same subsystem from being scheduled simultaneously.

        Note that the scheduler determines the requirements of a command when it is scheduled, so
        this method should normally be called from the command's constructor.

        :param requirements: the requirements to add
        """
        self.requirements.update(requirements)

    def runsWhenDisabled(self) -> bool:
        """
        Whether the given command should run when the robot is disabled. Override to return true if the
        command should run when disabled.

        :returns: whether the command should run when the robot is disabled
        """
        return False

    def schedule(self, interruptible: bool = True) -> None:
        """Schedules this command."""
        from .commandscheduler import CommandScheduler

        CommandScheduler.getInstance().schedule(self)

    def cancel(self) -> None:
        """
        Cancels this command. Will call #end(boolean) end(true). Commands will be canceled
        regardless of InterruptionBehavior interruption behavior.
        """
        from .commandscheduler import CommandScheduler

        CommandScheduler.getInstance().cancel(self)

    def isScheduled(self) -> bool:
        """
        Whether the command is currently scheduled. Note that this does not detect whether the command
        is in a composition, only whether it is directly being run by the scheduler.

        :returns: Whether the command is scheduled.
        """
        from .commandscheduler import CommandScheduler

        return CommandScheduler.getInstance().isScheduled(self)

    def hasRequirement(self, requirement: Subsystem) -> bool:
        """
        Whether the command requires a given subsystem.

        :param requirement: the subsystem to inquire about
        :returns: whether the subsystem is required
        """
        return requirement in self.requirements

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        """
        How the command behaves when another command with a shared requirement is scheduled.

        :returns: a variant of InterruptionBehavior, defaulting to {@link InterruptionBehavior#kCancelSelf kCancelSelf}.
        """
        return InterruptionBehavior.kCancelSelf

    def withTimeout(self, seconds: float) -> ParallelRaceGroup:
        """
        Decorates this command with a timeout. If the specified timeout is exceeded before the command
        finishes normally, the command will be interrupted and un-scheduled. Note that the timeout only
        applies to the command returned by this method; the calling command is not itself changed.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param seconds: the timeout duration
        :returns: the command with the timeout added
        """
        from .waitcommand import WaitCommand

        return self.raceWith(WaitCommand(seconds))

    def until(self, condition: Callable[[], bool]) -> ParallelRaceGroup:
        """
        Decorates this command with an interrupt condition. If the specified condition becomes true
        before the command finishes normally, the command will be interrupted and un-scheduled.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param condition: the interrupt condition
        :returns: the command with the interrupt condition added
        """
        from .waituntilcommand import WaitUntilCommand

        return self.raceWith(WaitUntilCommand(condition))

    def onlyWhile(self, condition: Callable[[], bool]) -> ParallelRaceGroup:
        """
        Decorates this command with a run condition. If the specified condition becomes false before
        the command finishes normally, the command will be interrupted and un-scheduled.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param condition: the interrupt condition
        :returns: the command with the interrupt condition added
        """
        return self.until(lambda: not condition())

    def withInterrupt(self, condition: Callable[[], bool]) -> ParallelRaceGroup:
        """
        Decorates this command with an interrupt condition. If the specified condition becomes true
        before the command finishes normally, the command will be interrupted and un-scheduled. Note
        that this only applies to the command returned by this method; the calling command is not
        itself changed.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param condition: the interrupt condition
        :returns: the command with the interrupt condition added
        @deprecated Replace with #until(BooleanSupplier)
        """
        return self.until(condition)

    def beforeStarting(self, before: Command) -> SequentialCommandGroup:
        """
        Decorates this command with another command to run before this command starts.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param before: the command to run before this one
        :returns: the decorated command
        """
        from .sequentialcommandgroup import SequentialCommandGroup

        return SequentialCommandGroup(before, self)

    def andThen(self, *next: Command) -> SequentialCommandGroup:
        """
        Decorates this command with a set of commands to run after it in sequence. Often more
        convenient/less-verbose than constructing a new SequentialCommandGroup explicitly.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param next: the commands to run next
        :returns: the decorated command
        """
        from .sequentialcommandgroup import SequentialCommandGroup

        return SequentialCommandGroup(self, *next)

    def deadlineWith(self, *parallel: Command) -> ParallelDeadlineGroup:
        """
        Decorates this command with a set of commands to run parallel to it, ending when the calling
        command ends and interrupting all the others. Often more convenient/less-verbose than
        constructing a new ParallelDeadlineGroup explicitly.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param parallel: the commands to run in parallel
        :returns: the decorated command
        """
        from .paralleldeadlinegroup import ParallelDeadlineGroup

        return ParallelDeadlineGroup(self, *parallel)

    def alongWith(self, *parallel: Command) -> ParallelCommandGroup:
        """
        Decorates this command with a set of commands to run parallel to it, ending when the last
        command ends. Often more convenient/less-verbose than constructing a new {@link
        ParallelCommandGroup} explicitly.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param parallel: the commands to run in parallel
        :returns: the decorated command
        """
        from .parallelcommandgroup import ParallelCommandGroup

        return ParallelCommandGroup(self, *parallel)

    def raceWith(self, *parallel: Command) -> ParallelRaceGroup:
        """
        Decorates this command with a set of commands to run parallel to it, ending when the first
        command ends. Often more convenient/less-verbose than constructing a new {@link
        ParallelRaceGroup} explicitly.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param parallel: the commands to run in parallel
        :returns: the decorated command
        """
        from .parallelracegroup import ParallelRaceGroup

        return ParallelRaceGroup(self, *parallel)

    def perpetually(self) -> PerpetualCommand:
        """
        Decorates this command to run perpetually, ignoring its ordinary end conditions. The decorated
        command can still be interrupted or canceled.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :returns: the decorated command
        @deprecated PerpetualCommand violates the assumption that execute() doesn't get called after
            isFinished() returns true -- an assumption that should be valid. This was unsafe/undefined
            behavior from the start, and RepeatCommand provides an easy way to achieve similar end
            results with slightly different (and safe) semantics.
        """
        from .perpetualcommand import PerpetualCommand

        return PerpetualCommand(self)

    def repeatedly(self) -> RepeatCommand:
        """
        Decorates this command to run repeatedly, restarting it when it ends, until this command is
        interrupted. The decorated command can still be canceled.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :returns: the decorated command
        """
        from .repeatcommand import RepeatCommand

        return RepeatCommand(self)

    def asProxy(self) -> ProxyCommand:
        """
        Decorates this command to run "by proxy" by wrapping it in a ProxyCommand. This is
        useful for "forking off" from command compositions when the user does not wish to extend the
        command's requirements to the entire command composition.

        :returns: the decorated command
        """
        from .proxycommand import ProxyCommand

        return ProxyCommand(self)

    def unless(self, condition: Callable[[], bool]) -> ConditionalCommand:
        """
        Decorates this command to only run if this condition is not met. If the command is already
        running and the condition changes to true, the command will not stop running. The requirements
        of this command will be kept for the new conditional command.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param condition: the condition that will prevent the command from running
        :returns: the decorated command
        """
        from .conditionalcommand import ConditionalCommand
        from .instantcommand import InstantCommand

        return ConditionalCommand(InstantCommand(), self, condition)

    def onlyIf(self, condition: Callable[[], bool]) -> ConditionalCommand:
        """
        Decorates this command to only run if this condition is met. If the command is already running
        and the condition changes to false, the command will not stop running. The requirements of this
        command will be kept for the new conditional command.

        Note: This decorator works by adding this command to a composition. The command the
        decorator was called on cannot be scheduled independently or be added to a different
        composition (namely, decorators), unless it is manually cleared from the list of composed
        commands with CommandScheduler#removeComposedCommand(Command). The command composition
        returned from this method can be further decorated without issue.

        :param condition: the condition that will allow the command to run
        :returns: the decorated command
        """
        return self.unless(lambda: not condition())

    def ignoringDisable(self, doesRunWhenDisabled: bool) -> WrapperCommand:
        """
        Decorates this command to run or stop when disabled.

        :param doesRunWhenDisabled: true to run when disabled.
        :returns: the decorated command
        """
        from .wrappercommand import WrapperCommand

        class W(WrapperCommand):
            def runsWhenDisabled(self) -> bool:
                return doesRunWhenDisabled

        return W(self)

    def withInterruptBehavior(self, behavior: InterruptionBehavior) -> WrapperCommand:
        """
        Decorates this command to have a different InterruptionBehavior interruption behavior.

        :param interruptBehavior: the desired interrupt behavior
        :returns: the decorated command
        """
        from .wrappercommand import WrapperCommand

        class W(WrapperCommand):
            def getInterruptionBehavior(self) -> InterruptionBehavior:
                return behavior

        return W(self)

    def finallyDo(self, end: Callable[[bool], Any]) -> WrapperCommand:
        """
        Decorates this command with a lambda to call on interrupt or end, following the command's
        inherent #end(boolean) method.

        :param end: a lambda accepting a boolean parameter specifying whether the command was
            interrupted.
        :return: the decorated command
        """
        from .wrappercommand import WrapperCommand

        class W(WrapperCommand):
            def end(self, interrupted: bool) -> None:
                self._command.end(interrupted)
                end(interrupted)

        return W(self)

    def handleInterrupt(self, handler: Callable[[], Any]) -> WrapperCommand:
        """
        Decorates this command with a lambda to call on interrupt, following the command's inherent
        #end(boolean) method.

        :param handler: a lambda to run when the command is interrupted
        :returns: the decorated command
        """
        return self.finallyDo(lambda interrupted: handler() if interrupted else None)

    def getName(self) -> str:
        """
        Gets the name of this Command.

        :returns: Name
        """
        return SendableRegistry.getName(self)

    def setName(self, name: str):
        """
        Sets the name of this Command.

        :param name: Name
        """
        SendableRegistry.setName(self, name)

    def withName(self, name: str) -> WrapperCommand:
        """
        Decorates this command with a name.

        :param name: the name of the command
        :returns: the decorated command
        """
        from .wrappercommand import WrapperCommand

        wrapper = WrapperCommand(self)
        wrapper.setName(name)
        return wrapper

    def initSendable(self, builder: SendableBuilder) -> None:
        from .commandscheduler import CommandScheduler

        builder.setSmartDashboardType("Command")
        builder.addStringProperty(
            ".name",
            self.getName,
            lambda _: None,
        )

        def on_set(value: bool) -> None:
            if value:
                if not self.isScheduled():
                    self.schedule()
            else:
                if self.isScheduled():
                    self.cancel()

        builder.addBooleanProperty(
            "running",
            self.isScheduled,
            on_set,
        )
        builder.addBooleanProperty(
            ".isParented",
            lambda: CommandScheduler.getInstance().isComposed(self),
            lambda _: None,
        )
        builder.addStringProperty(
            "interruptBehavior",
            lambda: self.getInterruptionBehavior().name,
            lambda _: None,
        )
        builder.addBooleanProperty(
            "runsWhenDisabled",
            self.runsWhenDisabled,
            lambda _: None,
        )
