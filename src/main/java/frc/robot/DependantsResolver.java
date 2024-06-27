package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DependantsResolver {
    public static <A, K> K collapse(Optional<A> a, PositiveResolution1<A, K> positive, EvilResoultion<K> negative) {
        if(a.isPresent()) {
            return positive.run(a.get());
        }
        return negative.run();
    }

    public static <A> Command collapse(Optional<A> a, PositiveResolution1<A, Command> positive) {
        if(a.isPresent()) {
            return positive.run(a.get());
        }
        return new InstantCommand();
    }

    public static <A, B, K> K collapse(Optional<A> a, Optional<B> b, PositiveResolution2<A, B, K> positive, EvilResoultion<K> negative) {
        if(a.isPresent() && b.isPresent()) {
            return positive.run(a.get(), b.get());
        }
        return negative.run();
    }

    public static <A, B> Command collapse(Optional<A> a, Optional<B> b, PositiveResolution2<A, B, Command> positive) {
        if(a.isPresent() && b.isPresent()) {
            return positive.run(a.get(), b.get());
        }
        return new InstantCommand();
    }

    public static <A, B, C, K> K collapse(Optional<A> a, Optional<B> b, Optional<C> c, PositiveResolution3<A,B,C,K> positive, EvilResoultion<K> negative) {
        if(a.isPresent() && b.isPresent() && c.isPresent()) {
            return positive.run(a.get(), b.get(), c.get());
        }
        return negative.run();
    }

    public static <A, B, C, D, K> K collapse(Optional<A> a, Optional<B> b, Optional<C> c, Optional<D> d, PositiveResolution4<A,B,C,D,K> positive, EvilResoultion<K> negative) {
        if(a.isPresent() && b.isPresent() && c.isPresent() && d.isPresent()) {
            return positive.run(a.get(), b.get(), c.get(), d.get());
        }
        return negative.run();
    }

    @FunctionalInterface
    interface PositiveResolution1<T, K> {
        K run(T first);
    }

    @FunctionalInterface
    interface PositiveResolution2 <A, B, K> {
        K run(A first, B second);
    }

    @FunctionalInterface
    interface PositiveResolution3<A, B, C, K> {
        K run(A first, B second, C third);
    }

    @FunctionalInterface
    interface PositiveResolution4<A, B, C, D, K> {
        K run(A first, B second, C third, D fourth);
    }

    @FunctionalInterface
    interface EvilResoultion<T> {
        T run();
    }
}
