package frc.robot;

public class TypedApriltag {
  public final int id;

  public TypedApriltag(int id) {
    this.id = id;
  }

  public boolean isEnemyApriltag(TeamAffiliation robotTeam) {
    return !isFriendlyApriltag(robotTeam);
  }

  public boolean isFriendlyApriltag(TeamAffiliation robotTeam) {
    boolean isBlue = Constants.ApriltagConstants.blue.contains(id);
    if (robotTeam == TeamAffiliation.BLUE) {
      return isBlue;
    } else {
      return !isBlue;
    }
  }

  public boolean isSource() {
    return Constants.ApriltagConstants.source.contains(id);
  }

  public boolean isAmp() {
    return Constants.ApriltagConstants.amp.contains(id);
  }

  public boolean isSpeaker() {
    return Constants.ApriltagConstants.speaker.contains(id);
  }

  enum TeamAffiliation {
    RED,
    BLUE
  }
}
